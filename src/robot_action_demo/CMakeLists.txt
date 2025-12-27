cmake_minimum_required(VERSION 3.8)
project(robot_action_demo)

# =========================
# 基础依赖
# =========================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# =========================
# Action 接口生成
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveRobot.action"
)

# =========================
# 安装 Python 可执行节点
# （无 .py 后缀，ros2 run 友好）
# =========================
install(PROGRAMS
  robot_action_demo/move_robot_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_robot_server
)

install(PROGRAMS
  robot_action_demo/move_robot_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_robot_client
)

# =========================
# ament package
# =========================
ament_package()