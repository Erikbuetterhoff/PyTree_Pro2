#!/usr/bin/env python3


# Node that publishes true / false depending if "t" was pressed or not
# topicname = bt_test_topic

import rclpy
from std_msgs.msg import Bool
from pynput import keyboard

def main():
    global mypub, mymsg
    mymsg = Bool()
    
    rclpy.init()
    test_pub_node = rclpy.create_node('test_pub_node')
    mypub = test_pub_node.create_publisher(Bool, "bt_test_topic", 1)
    test_pub_node.create_timer(0.1, mytimercallback)
    
    # Start listening to keyboard events
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    
    try:
        rclpy.spin(test_pub_node)
    except KeyboardInterrupt:
        pass

    test_pub_node.destroy_node()
    rclpy.shutdown()
    listener.stop()

def on_press(key):
    try:
        if key.char == 't':
            change_value()
    except AttributeError:
        pass

def change_value():
    global mymsg
    mymsg.data = not mymsg.data

def mytimercallback():
    global mypub, mymsg
    mypub.publish(mymsg)

if __name__ == '__main__':
    main()