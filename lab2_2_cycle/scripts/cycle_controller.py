#!/usr/bin/env python3
"""Follow the warehouse cycle using map-aligned measured wheel odometry."""
import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion


# x, y, service operation (if any). Begin at the loading station.
ROUTE = (
    (-1.10, 0.00, 'loading'),
    (-1.10, 0.90, ''),
    (1.10, 0.90, ''),
    (1.10, 0.00, 'unloading'),
    (1.10, -0.90, ''),
    (-1.10, -0.90, ''),
)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class CycleController:
    def __init__(self):
        self.pose = None
        self.target = 0
        self.aligned = False
        self.dwell_until = None
        self.dwell = rospy.Duration.from_sec(rospy.get_param('~dwell', 2.0))
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.subscriber = rospy.Subscriber(
            '/warehouse/pose', PoseStamped, self.read_pose, queue_size=1
        )
        rospy.on_shutdown(self.stop)

    def read_pose(self, msg):
        self.pose = msg

    def stop(self):
        self.publisher.publish(Twist())

    def step(self, now):
        command = Twist()
        pose = self.pose
        if pose is None or not 0.0 <= (now - pose.header.stamp).to_sec() <= 0.35:
            return command

        if self.dwell_until is not None:
            if now >= self.dwell_until:
                self.advance()
            return command

        x, y, service = ROUTE[self.target]
        dx = x - pose.pose.position.x
        dy = y - pose.pose.position.y
        distance = math.hypot(dx, dy)
        if distance <= 0.025:
            if service:
                rospy.loginfo('At %s station, holding for %.1f s', service, self.dwell.to_sec())
                self.dwell_until = now + self.dwell
            else:
                self.advance()
            return command

        q = pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        error = math.atan2(dy, dx) - yaw
        error = math.atan2(math.sin(error), math.cos(error))
        if abs(error) > 0.25:
            self.aligned = False
        elif abs(error) <= 0.05:
            self.aligned = True

        command.angular.z = clamp(2.4 * error, 0.8)
        if self.aligned:
            command.linear.x = min(0.15, 1.2 * distance) * math.cos(error)
        return command

    def advance(self):
        self.target = (self.target + 1) % len(ROUTE)
        self.aligned = False
        self.dwell_until = None

    def run(self):
        rate = rospy.Rate(30, reset=True)
        while not rospy.is_shutdown():
            self.publisher.publish(self.step(rospy.Time.now()))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('cycle_controller')
    CycleController().run()
