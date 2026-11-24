#!/usr/bin/env python3
"""Anchor wheel odometry to the known starting pose on the warehouse map."""
import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class WarehouseLocalization:
    def __init__(self):
        self.x = rospy.get_param('~start_x', -1.10)
        self.y = rospy.get_param('~start_y', 0.0)
        self.yaw = rospy.get_param('~start_yaw', 0.0)
        self.cos_yaw = math.cos(self.yaw)
        self.sin_yaw = math.sin(self.yaw)

        self.tf = tf2_ros.StaticTransformBroadcaster()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = quaternion_from_euler(0.0, 0.0, self.yaw)
        self.tf.sendTransform(transform)

        self.publisher = rospy.Publisher('/warehouse/pose', PoseStamped, queue_size=10)
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.update, queue_size=10)

    def update(self, msg):
        odom_pose = msg.pose.pose
        x, y = odom_pose.position.x, odom_pose.position.y
        q = odom_pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))

        result = PoseStamped()
        result.header.stamp = msg.header.stamp
        result.header.frame_id = 'map'
        result.pose.position.x = self.x + self.cos_yaw * x - self.sin_yaw * y
        result.pose.position.y = self.y + self.sin_yaw * x + self.cos_yaw * y
        q = result.pose.orientation
        q.x, q.y, q.z, q.w = quaternion_from_euler(0.0, 0.0, self.yaw + yaw)
        self.publisher.publish(result)


if __name__ == '__main__':
    rospy.init_node('localization_publisher')
    node = WarehouseLocalization()
    rospy.spin()
