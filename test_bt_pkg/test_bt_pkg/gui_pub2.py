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
        self.timer = None

    def create_publisher_for_topic(self, topic_name):
        self.publisher_ = self.create_publisher(Bool, topic_name, 10)
        self.get_logger().info(f"Publisher created for topic: {topic_name}")

    def publish_state(self):
        if self.publisher_ is not None:
            self.get_logger().info(f'Publishing: {self.state}')
            msg = Bool()
            msg.data = self.state
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Publisher is not created. Please select a topic.")

    def toggle_state(self):
        self.state = not self.state
        self.get_logger().info(f'Toggled state to: {self.state}')

    def get_available_topics(self):
        topics = self.get_topic_names_and_types()
        bool_topics = [name for name, types in topics if 'std_msgs/msg/Bool' in types]
        return bool_topics

    def start_publishing(self):
        self.timer = self.create_timer(1.0, self.publish_state)  # Publish every 1 second
        self.get_logger().info("Started publishing")

    def stop_publishing(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            self.get_logger().info("Stopped publishing")

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

        self.toggle_button = tk.Button(root, text="Toggle State", command=self.toggle_state)
        self.toggle_button.pack(pady=10)

        self.start_button = tk.Button(root, text="Start Publishing", command=self.start_publishing)
        self.start_button.pack(pady=10)

        self.stop_button = tk.Button(root, text="Stop Publishing", command=self.stop_publishing)
        self.stop_button.pack(pady=10)

        self.state_label = tk.Label(root, text="Current State: False")
        self.state_label.pack(pady=10)

        self.refresh_topics()

    def refresh_topics(self):
        available_topics = self.node.get_available_topics()
        self.topic_dropdown['values'] = available_topics
        if available_topics:
            self.topic_dropdown.current(0)

    def select_topic(self):
        selected_topic = self.topic_var.get()
        self.node.create_publisher_for_topic(selected_topic)

    def toggle_state(self):
        self.node.toggle_state()
        self.state_label.config(text=f"Current State: {self.node.state}")

    def start_publishing(self):
        self.node.start_publishing()

    def stop_publishing(self):
        self.node.stop_publishing()

def main(args=None):
    rclpy.init(args=args)
    node = BoolPublisherNode()

    root = tk.Tk()
    root.title("Bool Publisher")
    root.geometry("400x350")

    app = BoolPublisherApp(root, node)

    def update_ros():
        rclpy.spin_once(node, timeout_sec=0.1)
        root.after(50, update_ros)

    root.after(50, update_ros)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
