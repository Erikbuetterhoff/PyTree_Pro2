#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Bool
import keyboard

def main():
    global mypub
    rclpy.init()
    test_pub_node = rclpy.create_node('test_pub_node')
    mypub = test_pub_node.create_publisher(Bool, "bt_test_topic", 1)
    test_pub_node.create_timer(0.1, mytimercallback)
    try:
        rclpy.spin(test_pub_node)
    except KeyboardInterrupt:
        pass

    test_pub_node.destroy_node()
    rclpy.shutdown()

def change_value():
    mymsg = not mymsg

keyboard.add_hotkey("t", change_value)

def mytimercallback():
    global mypub
    global mymsg 
    mymsg = Bool()
    mypub.publish(mymsg)

if __name__ == '__main__':
    main()