#! /usr/bin/env python3

import rclpy
import threading
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

        self.fr_pub = self.create_publisher(Float64MultiArray, '/right_front_controller/commands', 10)
        self.fl_pub = self.create_publisher(Float64MultiArray, '/left_front_controller/commands', 10)
        self.rr_pub = self.create_publisher(Float64MultiArray, '/right_rear_controller/commands', 10)
        self.rl_pub = self.create_publisher(Float64MultiArray, '/left_rear_controller/commands', 10)

    def callback_data(self, msg):

        cmd_vel = np.matrix([msg.linear.x, msg.linear.y, msg.angular.z])

        wheel_vel = (np.dot(mat, cmd_vel.T).A1).tolist()

        wv = Float64MultiArray()

        wv.data = [5*wheel_vel[0]]
        self.fr_pub.publish(wv)

        wv.data = [5*wheel_vel[1]]
        self.fl_pub.publish(wv)

        wv.data = [5*wheel_vel[2]]
        self.rr_pub.publish(wv)

        wv.data = [5*wheel_vel[3]]
        self.rl_pub.publish(wv)
        #
        # dF = (wheel_vel[0]+wheel_vel[1]+wheel_vel[2]+wheel_vel[3])/4
        # dR = (-wheel_vel[0]+wheel_vel[1]+wheel_vel[2]-wheel_vel[3])/4
        #
        # print(dF, dR)

def main(args=None):
    rclpy.init(args=args)
    mecanum_node = MecanumNode()
    rclpy.spin(mecanum_node)

    # spinner = threading.Thread(target=rclpy.spin, args=(mecanum_node,))
    # spinner.start()


    mecanum_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()