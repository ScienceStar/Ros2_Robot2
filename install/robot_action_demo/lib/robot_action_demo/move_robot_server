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
        distance = goal_handle.request.distance
        self.get_logger().info(f'Executing goal: move {distance} units')

        feedback_msg = MoveRobot.Feedback()
        current_distance = 0.0
        step = 0.1

        # 模拟移动，每秒输出一次 feedback
        while current_distance < distance and rclpy.ok():
            current_distance += step
            if current_distance > distance:
                current_distance = distance

            feedback_msg.current_distance = current_distance
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {current_distance:.2f}')
            time.sleep(1.0)  # 每秒输出一次

        # Goal 完成
        goal_handle.succeed()
        result = MoveRobot.Result()
        result.success = True
        self.get_logger().info('Goal completed!')

        return result

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