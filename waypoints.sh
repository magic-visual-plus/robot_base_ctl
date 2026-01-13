python3 streaming_pose_servo_waypoints.py --ros-args \
  -p odom_topic:=/rko_lio/odometry \
  -p cmd_topic:=/cmd_vel \
  -p v_ref:=0.2 \
  -p hold_heading:=False \
  -p k_xy:=0.35 \
  -p max_correction_v:=0.20 \
  -p out_csv:=/opt/project/robot_base_ctl/base_ctl/Threepoints.csv
