cmake_minimum_required(VERSION 3.8)
project(phi_motion_torso)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToHeight.action"
  "msg/TorsoStatus.msg"
  "msg/TorsoExtend.msg"
  "srv/Initialize.srv"
  "srv/GetMotionParams.srv"
  "srv/SetMotionParams.srv"
  DEPENDENCIES std_msgs action_msgs builtin_interfaces
)

# Install Python modules (rosidl already installs generated msg/srv/action;
# here we install our node/package code into the standard Python path)
_ament_cmake_python_register_environment_hook()
install(DIRECTORY phi_motion_torso/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS
  phi_motion_torso/torso_node.py
  phi_motion_torso/torso_teleop_node.py
  phi_motion_torso/torso_teleop_follow_topic_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install shared python data model (tosor_ctl/data_model) so runtime imports work.
install(DIRECTORY ../data_model/
  DESTINATION "${PYTHON_INSTALL_DIR}/data_model"
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
