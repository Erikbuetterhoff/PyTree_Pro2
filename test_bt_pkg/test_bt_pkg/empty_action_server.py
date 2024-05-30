import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import time

import action_pkg.action as Actions


class EmptyActionServer(Node):          #ros2 action send_goal test_action action_pkg/action/Test "{numbers: 5}"

    def __init__(self):
        super().__init__('empty_action_server')
        self._action_server = ActionServer(
            self,
            Actions.Empty,
            'empty_action', #Name unter dem die Action aufgerufen wird
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Actions.Empty.Feedback()
        
        i=5
        while i > 0:
            feedback_msg.part_result = i
            self.get_logger().info(f"Feedback: {feedback_msg.part_result}")
            goal_handle.publish_feedback(feedback_msg)
            i = i - 1
            time.sleep(1)
        
        result = Actions.Empty.Result()
        result.result = feedback_msg.part_result
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    empty_action_server = EmptyActionServer()

    rclpy.spin(empty_action_server)


if __name__ == '__main__':
    main()