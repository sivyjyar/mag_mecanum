#! /usr/bin/env python3


import rclpy
import sys

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

from tf2_ros.transform_broadcaster import TransformBroadcaster
from nav_msgs.msg import Odometry


class DynamicBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_broadcaster')
        self.tfb_ = TransformBroadcaster(self)
        self.sub_pose = self.create_subscription(Odometry,'/odom', self.handle_pose, 10)

    def handle_pose(self, msg):
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = 'odom'
        tfs._child_frame_id = 'base_link'

        tfs.transform.translation.x = msg.pose.pose.position.x
        tfs.transform.translation.y = msg.pose.pose.position.y
        tfs.transform.translation.z = msg.pose.pose.position.z

        tfs.transform.rotation.x = msg.pose.pose.orientation.x
        tfs.transform.rotation.y = msg.pose.pose.orientation.y
        tfs.transform.rotation.z = msg.pose.pose.orientation.z
        tfs.transform.rotation.w = msg.pose.pose.orientation.w

        self.tfb_.sendTransform(tfs)


def main(args=None):
    rclpy.init()
    node = DynamicBroadcaster()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()












# from geometry_msgs.msg import TransformStamped
#
# import rclpy
# from rclpy.node import Node
#
# from tf2_ros import TransformBroadcaster
#
# from nav_msgs.msg import Odometry
#
#
#
#
# class FramePublisher(Node):
#
#     def __init__(self):
#         super().__init__('tf2_frame_publisher')
#
#         # Initialize the transform broadcaster
#         self.tf_broadcaster = TransformBroadcaster(self)
#
#         self.subscription = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.handle_pose,
#             10)
#         self.subscription  # prevent unused variable warning
#
#     def handle_pose(self, msg):
#         t = TransformStamped()
#
#         # Read message content and assign it to
#         # corresponding tf variables
#         t.header.stamp = self.get_clock().now().to_msg()
#         # t.header.stamp = self.get_clock().to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#
#
#         t.transform.translation.x = msg.pose.pose.position.x
#         t.transform.translation.y = msg.pose.pose.position.y
#         t.transform.translation.z = msg.pose.pose.position.z
#
#
#         t.transform.rotation.x = msg.pose.pose.orientation.x
#         t.transform.rotation.y = msg.pose.pose.orientation.y
#         t.transform.rotation.z = msg.pose.pose.orientation.z
#         t.transform.rotation.w = msg.pose.pose.orientation.w
#
#         print(t)
#         # Send the transformation
#         self.tf_broadcaster.sendTransform(t)
#
# def main(args=None):
#     rclpy.init()
#     node = FramePublisher()
#
#     rclpy.spin(node)
#
#     node.destroy_node()
#     rclpy.shutdown()
#
#
#
# if __name__ == '__main__':
#     main()