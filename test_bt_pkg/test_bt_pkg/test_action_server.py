import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import time

import action_pkg.action as Actions


class TestActionServer(Node):          #ros2 action send_goal test_action action_pkg/action/Test "{numbers: 5}"

    def __init__(self):
        super().__init__('test_action_server')
        self._action_server = ActionServer(
            self,
            Actions.Test,
            'test_action', #Name unter dem die Action aufgerufen wird
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Actions.Test.Feedback()

        input = goal_handle.request.numbers
        while input > 0:
            feedback_msg.part_result = input
            self.get_logger().info(f"Feedback: {feedback_msg.part_result}")
            goal_handle.publish_feedback(feedback_msg)
            input = input - 1
            time.sleep(1)
        
        result = Actions.Test.Result()
        result.result = feedback_msg.part_result
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    test_action_server = TestActionServer()

    rclpy.spin(test_action_server)


if __name__ == '__main__':
    main()