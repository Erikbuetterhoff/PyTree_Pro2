import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_pkg.action import Test

class TestActionServer(Node):          #ros2 action send_goal test_action action_pkg/action/Test "{numbers: 5}"

    def __init__(self):
        super().__init__('test_action_server')
        self._action_server = ActionServer(
            self,
            Test,
            'test_action', #Name unter dem die Action aufgerufen wird
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Test.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    test_action_server = TestActionServer()

    rclpy.spin(test_action_server)


if __name__ == '__main__':
    main()