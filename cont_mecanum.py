#! /usr/bin/env python3

import rclpy
import time
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray

width = {"fr":0.275, "fl":0.275, "rr":0.275, "rl":0.275}
length = {"fr":0.575, "fl":0.575, "rr":0.575, "rl":0.575}

mat = np.matrix([[1, 1, (width["fr"] + length["fr"])],
                 [1, -1, -(width["fl"] + length["fl"])],
                 [1, -1, (width["rr"] + length["rr"])],
                 [1, 1, -(width["rl"] + length["rl"])]])


class MecanumNode(Node):


    def __init__(self):
        super().__init__('mecanum_node')

        # SUBSCRIBER
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback_data,
            10)
        # # self.subscription



        self.fr_pub = self.create_publisher(Float64MultiArray, '/right_front_controller/commands', 10)
        self.fl_pub = self.create_publisher(Float64MultiArray, '/left_front_controller/commands', 10)
        self.rr_pub = self.create_publisher(Float64MultiArray, '/right_rear_controller/commands', 10)
        self.rl_pub = self.create_publisher(Float64MultiArray, '/left_rear_controller/commands', 10)

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


    def callback_data(self, msg):

        cmd_vel = np.matrix([msg.linear.x, msg.linear.y, msg.angular.z])

        wheel_vel = (np.dot(mat, cmd_vel.T).A1).tolist()

        wv = Float64MultiArray()

        wv.data = [wheel_vel[0]]
        self.fr_pub.publish(wv)

        wv.data = [wheel_vel[1]]
        self.fl_pub.publish(wv)

        wv.data = [wheel_vel[2]]
        self.rr_pub.publish(wv)

        wv.data = [wheel_vel[3]]
        self.rl_pub.publish(wv)

        # new_msg = Float64MultiArray()
        # new_msg.data = [msg.linear.x]
        # print(msg.linear.x)



        # self.get_logger().info("Received: ")
        # self.publisher.publish(new_msg)

    # def listener_callback(self, msg):
    #     lin_x = Vector3()
    #     lin_x.x = msg.linear.x
    #     print(lin_x)
    #     self.get_logger().info('guudddd')
    #
    # def timer_callback(self):
    #     msg = Float64MultiArray()
    #     msg.data = lin_x
    #     self.publisher.publish(msg.data)
    #     self.get_logger().info('Publishing:')

def main(args=None):
    rclpy.init(args=args)
    mecanum_node = MecanumNode()
    rclpy.spin(mecanum_node)


    mecanum_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()