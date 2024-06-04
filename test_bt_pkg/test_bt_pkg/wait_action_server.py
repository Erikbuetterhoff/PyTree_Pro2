import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import time

import action_pkg.action as actions


class WaitActionServer(Node):          #ros2 action send_goal wait_action action_pkg/action/Wait "{timer: 15}"

    def __init__(self):
        super().__init__('wait_action_server')
        self._action_server = ActionServer(
            self,
            actions.Wait,
            'wait_action', #Name unter dem die Action aufgerufen wird
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = actions.Wait.Feedback()

        input = goal_handle.request.timer
        while input > 0:
            feedback_msg.part_result = input
            self.get_logger().info(f"Restzeit: {feedback_msg.part_result}s")
            goal_handle.publish_feedback(feedback_msg)
            input = input - 1
            time.sleep(1)
        
        result = actions.Wait.Result()
        result.result = f"Timer abgelaufen (Restzeit: {feedback_msg.part_result})"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    wait_action_server = WaitActionServer()

    rclpy.spin(wait_action_server)


if __name__ == '__main__':
    main()