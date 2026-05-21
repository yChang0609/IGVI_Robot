cmake_minimum_required(VERSION 3.8)
project(wildbot_grasp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GrabObject.action"
)

ament_python_install_package(wildbot_grasp_nodes)

install(PROGRAMS
  scripts/grab_object_server
  scripts/gripper_command
  scripts/arm_safeguard_node
  scripts/bear_grasp_task_node
  scripts/bear_nav_interface_node
  scripts/grasp_stack
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
