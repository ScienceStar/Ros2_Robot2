#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from robot_action_demo.action import MoveRobot
import time

class MoveRobotActionServer(Node):

    def __init__(self):
        super().__init__('move_robot_action_server')
        self._action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            self.execute_callback
        )
        self.get_logger().info("MoveRobot Action Server started, waiting for goals...")

    def execute_callback(self, goal_handle):
        """
        无限执行 Goal，每秒输出一次 Feedback
        Goal 永不完成（客户端可以持续接收 Feedback）
        """
        distance = goal_handle.request.distance
        self.get_logger().info(f'Executing goal: move {distance} units (infinite loop)')

        feedback_msg = MoveRobot.Feedback()
        current_distance = 0.0
        step = 0.1

        # 无限循环，不结束 Goal
        while rclpy.ok():
            current_distance += step
            feedback_msg.current_distance = current_distance
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {current_distance:.2f}')
            time.sleep(1.0)  # 每秒输出一次

        # 永远不结束 Goal
        # 如果你想在某些条件下完成 Goal，可以调用 goal_handle.succeed()

        return None  # 注意这里不要 return result

def main(args=None):
    rclpy.init(args=args)
    server = MoveRobotActionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()