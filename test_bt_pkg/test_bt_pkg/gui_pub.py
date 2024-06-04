#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tkinter as tk
from tkinter import ttk

class BoolPublisherNode(Node):
    def __init__(self):
        super().__init__('bool_publisher_node')
        self.publisher_ = None
        self.state = False

    def create_publisher_for_topic(self, topic_name):
        self.publisher_ = self.create_publisher(Bool, topic_name, 10)
        self.get_logger().info(f"Publisher created for topic: {topic_name}")

    def publish_state(self):
        if self.publisher_ is not None:
            self.state = not self.state
            self.get_logger().info(f'Publishing: {self.state}')
            msg = Bool()
            msg.data = self.state
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Publisher is not created. Please select a topic.")

    def get_available_topics(self):
        topics = self.get_topic_names_and_types()
        bool_topics = [name for name, types in topics if 'std_msgs/msg/Bool' in types]
        return bool_topics

class BoolPublisherApp:
    def __init__(self, root, node):
        self.node = node

        self.topic_var = tk.StringVar()
        self.topic_dropdown = ttk.Combobox(root, textvariable=self.topic_var)
        self.topic_dropdown.pack(pady=10)

        self.refresh_button = tk.Button(root, text="Refresh Topics", command=self.refresh_topics)
        self.refresh_button.pack(pady=10)

        self.select_button = tk.Button(root, text="Select Topic", command=self.select_topic)
        self.select_button.pack(pady=10)

        self.publish_button = tk.Button(root, text="Toggle", command=self.toggle_publish)
        self.publish_button.pack(pady=10)

        self.refresh_topics()

    def refresh_topics(self):
        available_topics = self.node.get_available_topics()
        self.topic_dropdown['values'] = available_topics
        if available_topics:
            self.topic_dropdown.current(0)

    def select_topic(self):
        selected_topic = self.topic_var.get()
        self.node.create_publisher_for_topic(selected_topic)

    def toggle_publish(self):
        self.node.publish_state()

def main(args=None):
    rclpy.init(args=args)
    node = BoolPublisherNode()

    root = tk.Tk()
    root.title("Bool Publisher")
    app = BoolPublisherApp(root, node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
