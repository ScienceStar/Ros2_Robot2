#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_action_demo.action import MoveRobot

class MoveRobotActionClient(Node):

    def __init__(self):
        super().__init__('move_robot_action_client')
        self._client = ActionClient(self, MoveRobot, 'move_robot')

    def send_goal(self, distance):
        self.get_logger().info(f'Sending goal: move {distance} units')
        goal_msg = MoveRobot.Goal()
        goal_msg.distance = distance

        self._client.wait_for_server()
        self._future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback received: {feedback_msg.feedback.current_distance:.2f}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = MoveRobotActionClient()
    client.send_goal(2.0)  # 示例移动距离 2 单位
    rclpy.spin(client)

if __name__ == '__main__':
    main()