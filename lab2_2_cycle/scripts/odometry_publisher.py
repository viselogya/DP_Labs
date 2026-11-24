#!/usr/bin/env python3
"""Integrate measured RoboCake drive-wheel positions."""
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler


class WheelOdometry:
    def __init__(self):
        # Match the geometry used by diff_drive_controller in warehouse.launch.
        self.radius = rospy.get_param('/diff_drive_controller/wheel_radius', 0.022131)
        self.track = rospy.get_param('/diff_drive_controller/wheel_separation', 0.084918)
        self.x = self.y = self.yaw = 0.0
        self.previous = None
        self.publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.update, queue_size=10)

    def update(self, msg):
        positions = dict(zip(msg.name, msg.position))
        left = positions.get('left_wheel_joint', math.nan)
        right = positions.get('right_wheel_joint', math.nan)
        stamp = msg.header.stamp
        if not (math.isfinite(left) and math.isfinite(right)) or stamp == rospy.Time():
            return

        linear = angular = 0.0
        if self.previous is not None:
            old_stamp, old_left, old_right = self.previous
            dt = (stamp - old_stamp).to_sec()
            if dt <= 0:
                return
            dl = self.radius * (left - old_left)
            dr = self.radius * (right - old_right)
            distance = (dl + dr) / 2.0
            turn = (dr - dl) / self.track
            # Integrate the circular arc, with the straight-line limit at zero turn.
            half_turn = turn / 2.0
            arc_scale = math.sin(half_turn) / half_turn if abs(half_turn) > 1e-8 else 1.0
            self.x += distance * arc_scale * math.cos(self.yaw + half_turn)
            self.y += distance * arc_scale * math.sin(self.yaw + half_turn)
            heading = self.yaw + turn
            self.yaw = math.atan2(math.sin(heading), math.cos(heading))
            linear = distance / dt
            angular = turn / dt
        self.previous = stamp, left, right

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = odom.pose.pose.orientation
        q.x, q.y, q.z, q.w = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        # Nominal variances only. Large values mark unobserved axes.
        for index, variance in enumerate((0.01, 0.01, 1e6, 1e6, 1e6, 0.03)):
            odom.pose.covariance[7 * index] = variance
        for index, variance in enumerate((0.01, 1e6, 1e6, 1e6, 1e6, 0.03)):
            odom.twist.covariance[7 * index] = variance
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = q
        self.tf.sendTransform(transform)
        self.publisher.publish(odom)


if __name__ == '__main__':
    rospy.init_node('odometry_publisher')
    node = WheelOdometry()
    rospy.spin()
