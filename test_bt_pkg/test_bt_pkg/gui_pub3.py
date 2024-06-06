#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Bool
import tkinter as tk
from tkinter import ttk

def main():
    global mypub, mymsg
    mymsg = Bool()

    rclpy.init()
    test_pub_node = rclpy.create_node('test_pub_node')
    mypub = test_pub_node.create_publisher(Bool, "bt_test_topic", 1)
    test_pub_node.create_timer(0.1, mytimercallback)

    root = tk.Tk()
    root.title("Bool Publisher")
    root.geometry("400x300")

    publish_button = tk.Button(root, text="Toggle", command=change_value)
    publish_button.pack(pady=10)

    def update_ros():
        rclpy.spin_once(test_pub_node, timeout_sec=0.1)
        root.after(100, update_ros)

    root.after(100, update_ros)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass

    test_pub_node.destroy_node()
    rclpy.shutdown()


def change_value():
    global mymsg
    mymsg.data = not mymsg.data

def mytimercallback():
    global mypub, mymsg
    mypub.publish(mymsg)

if __name__ == '__main__':
    main()
