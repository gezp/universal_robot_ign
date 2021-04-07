#!/usr/bin/python3
import sys

import rclpy
from std_msgs.msg import Bool

input_msg='''
#1 : close gripper to grasp.
#0 : open gripper.
input:
'''

def main():
    rclpy.init()
    node = rclpy.create_node('gripper_test')
    pub = node.create_publisher(Bool, '/gripper', 10)
    while(rclpy.ok()):
        # start detect
        mode = input(input_msg)
        mode = int(mode)
        msg=Bool()
        if mode == 1:
            msg.data = True
        elif mode == 0:
            msg.data = False
        pub.publish(msg)
        
if __name__ == '__main__':
    main()