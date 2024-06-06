#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tkinter as tk
from tkinter import ttk

# Globale Variablen
state = False
publisher = None
timer = None

def create_publisher_for_topic(node, topic_name):
    global publisher
    publisher = node.create_publisher(Bool, topic_name, 10)
    node.get_logger().info(f"Publisher created for topic: {topic_name}")

def publish_state(node):
    global state, publisher
    if publisher is not None:
        node.get_logger().info(f'Publishing: {state}')
        msg = Bool()
        msg.data = state
        publisher.publish(msg)
    else:
        node.get_logger().warn("Publisher is not created. Please select a topic.")

def set_state(new_state, node):
    global state
    state = new_state
    publish_state(node)

def start_publishing(node, interval_sec=0.1):
    global timer
    if timer is None:
        timer = node.create_timer(interval_sec, lambda: publish_state(node))
        node.get_logger().info("Started continuous publishing.")

def refresh_topics(node, dropdown):
    topics = node.get_topic_names_and_types()
    bool_topics = [name for name, types in topics if 'std_msgs/msg/Bool' in types]
    dropdown['values'] = bool_topics
    if bool_topics:
        dropdown.current(0)

def select_topic(node, dropdown):
    selected_topic = dropdown.get()
    create_publisher_for_topic(node, selected_topic)

def main(args=None):
    global state, publisher, timer
    state = False
    publisher = None
    timer = None

    rclpy.init(args=args)
    node = Node('bool_publisher_node')

    root = tk.Tk()
    root.title("Bool Publisher")
    root.geometry("400x300")

    topic_var = tk.StringVar()
    topic_dropdown = ttk.Combobox(root, textvariable=topic_var)
    topic_dropdown.pack(pady=10)

    refresh_button = tk.Button(root, text="Refresh Topics", command=lambda: refresh_topics(node, topic_dropdown))
    refresh_button.pack(pady=10)

    select_button = tk.Button(root, text="Select Topic", command=lambda: select_topic(node, topic_dropdown))
    select_button.pack(pady=10)

    publish_true_button = tk.Button(root, text="Publish True", command=lambda: set_state(True, node))
    publish_true_button.pack(pady=10)

    publish_false_button = tk.Button(root, text="Publish False", command=lambda: set_state(False, node))
    publish_false_button.pack(pady=10)

    refresh_topics(node, topic_dropdown)
    start_publishing(node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
