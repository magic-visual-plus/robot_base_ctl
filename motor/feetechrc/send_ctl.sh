# 使用 ros topic pub 简单测试下小车的控制指令

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: -0.5, z: 0.0}, angular: {z: 0.0}}"


ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0, z: 0.0}, angular: {z: 0.0}}"
