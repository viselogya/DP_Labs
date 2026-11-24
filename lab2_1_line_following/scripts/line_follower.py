#!/usr/bin/env python3
"""Follow RoboCake's dark stripe and stop on range or chassis contact."""
import math
from threading import Lock

import rospy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, String


def clamp(value, limit):
    return max(-limit, min(value, limit))


class LineFollower:
    STOP_DISTANCE = 0.10
    RESUME_DISTANCE = 0.12
    SENSOR_TIMEOUT = 0.35
    MAX_YAW = 2.0

    def __init__(self):
        self.kp = float(rospy.get_param("~kp", 1.5))
        self.ki = float(rospy.get_param("~ki", 0.08))
        self.kd = float(rospy.get_param("~kd", 0.10))
        self.speed = float(rospy.get_param("~speed", 0.10))
        self.threshold = float(rospy.get_param("~dark_threshold", 0.35))
        gains = (self.kp, self.ki, self.kd)
        if (not all(math.isfinite(gain) and gain >= 0 for gain in gains)
                or not 0 < self.speed <= 0.30
                or not 0 < self.threshold <= 1):
            raise ValueError("Use finite nonnegative gains, 0 < speed <= 0.30 and 0 < threshold <= 1")

        self.search_direction = -1 if rospy.get_param("~search_direction", -1) < 0 else 1
        self.lock = Lock()
        self.reflectance = {}
        self.front = None
        self.contact_time = None
        self.contact_latched = False
        self.obstacle = False
        self.seen_line = False
        self.lost_since = None
        self.state = None
        self.reset_pid()

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.error_pub = rospy.Publisher("~error", Float64, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber("/reflectance_" + side, Float64, self.on_reflectance,
                             callback_args=side, queue_size=1)
            for side in ("left", "center", "right")
        ]
        self.subscribers.extend([
            rospy.Subscriber("/proximity_front", Range, self.on_range, queue_size=1),
            rospy.Subscriber("/lab2/chassis_contacts", ContactsState, self.on_contact,
                             queue_size=1),
        ])
        self.timer = rospy.Timer(rospy.Duration(0.05), self.step, reset=True)
        rospy.on_shutdown(self.shutdown)

    def on_reflectance(self, msg, side):
        with self.lock:
            self.reflectance[side] = (msg.data, rospy.Time.now())

    def on_range(self, msg):
        distance = msg.range
        if distance == -math.inf:
            distance = 0.0
        with self.lock:
            self.front = (distance, rospy.Time.now())
            if 0 <= distance <= self.STOP_DISTANCE:
                self.obstacle = True
            elif distance >= self.RESUME_DISTANCE:
                self.obstacle = False

    def on_contact(self, msg):
        with self.lock:
            self.contact_time = rospy.Time.now()
            self.contact_latched |= bool(msg.states)

    def fresh(self, now, stamp):
        return stamp is not None and 0 <= (now - stamp).to_sec() <= self.SENSOR_TIMEOUT

    def reset_pid(self):
        self.previous_error = None
        self.previous_time = None
        self.integral = 0.0
        self.derivative = 0.0

    def publish(self, state, linear=0.0, yaw=0.0):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = yaw
        self.cmd_pub.publish(cmd)
        if state != self.state:
            self.state = state
            self.state_pub.publish(String(data=state))
            rospy.loginfo("Line follower: %s", state)

    def stop(self, state):
        self.reset_pid()
        self.lost_since = None
        self.publish(state)

    def search(self, now):
        self.reset_pid()
        if not self.seen_line:
            self.publish("waiting_for_line")
            return
        if self.lost_since is None or now < self.lost_since:
            self.lost_since = now
        elapsed = (now - self.lost_since).to_sec()
        if elapsed < 0.65:
            self.publish("searching", yaw=0.65 * self.search_direction)
        elif elapsed < 1.95:
            self.publish("searching", yaw=-0.65 * self.search_direction)
        else:
            self.publish("line_lost")

    def follow(self, now, error, distance):
        if self.previous_time is not None:
            dt = (now - self.previous_time).to_sec()
            if 0 < dt <= self.SENSOR_TIMEOUT:
                slope = (error - self.previous_error) / dt
                alpha = dt / (0.15 + dt)
                self.derivative += alpha * (slope - self.derivative)
                candidate = clamp(self.integral + error * dt, 0.8)
                effort = self.kp * error + self.ki * candidate + self.kd * self.derivative
                if abs(effort) <= self.MAX_YAW or effort * error < 0:
                    self.integral = candidate
            else:
                self.reset_pid()
        self.previous_error = error
        self.previous_time = now
        yaw = clamp(self.kp * error + self.ki * self.integral
                    + self.kd * self.derivative, self.MAX_YAW)
        corner_factor = 1 - 0.55 * abs(yaw) / self.MAX_YAW
        approach_factor = max(0.35, min(1.0, (distance - self.STOP_DISTANCE) / 0.10))
        self.error_pub.publish(Float64(data=error))
        self.publish("following", self.speed * corner_factor * approach_factor, yaw)

    def step(self, _event):
        with self.lock:
            now = rospy.Time.now()
            if self.contact_latched:
                self.stop("collision_latched")
                return
            if not self.front or not self.fresh(now, self.front[1]) or not self.fresh(now, self.contact_time):
                self.stop("waiting_for_sensors")
                return
            distance = self.front[0]
            # +Inf means clear. NaN or a negative finite reading must stop the robot.
            if not distance >= 0:
                self.stop("invalid_range")
                return
            if self.obstacle:
                self.stop("obstacle")
                return
            if len(self.reflectance) != 3 or any(
                not 0 <= value <= 1 or not self.fresh(now, stamp)
                for value, stamp in self.reflectance.values()
            ):
                self.stop("waiting_for_sensors")
                return

            left, center, right = (
                max(0.0, self.threshold - self.reflectance[side][0])
                for side in ("left", "center", "right")
            )
            weight = left + center + right
            if weight < 0.02:
                self.search(now)
                return

            error = (left - right) / weight
            if abs(error) > 0.1:
                self.search_direction = 1 if error > 0 else -1
            self.seen_line = True
            self.lost_since = None
            self.follow(now, error, distance)

    def shutdown(self):
        self.timer.shutdown()
        with self.lock:
            self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("line_follower")
    follower = LineFollower()
    rospy.spin()
