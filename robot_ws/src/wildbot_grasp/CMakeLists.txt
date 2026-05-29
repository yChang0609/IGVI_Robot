cmake_minimum_required(VERSION 3.8)
project(wildbot_grasp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GrabObject.action"
  "action/OpenDoor.action"
  "action/SearchAndRetrieve.action"
  "action/BridgeRetrieve.action"
  "action/BridgeTraverse.action"
)

ament_python_install_package(wildbot_grasp_nodes)

install(PROGRAMS
  scripts/grab_object_server
  scripts/gripper_command
  scripts/arm_safeguard_node
  scripts/open_door_server
  scripts/tune_open_door
  scripts/search_retrieve_server
  scripts/search_retrieve_launch
  scripts/bridge_retrieve_server
  scripts/bridge_retrieve_launch
  scripts/bridge_traverse_server
  scripts/bridge_traverse_launch
  scripts/arena_mission_server
  scripts/arena_mission_launch
  scripts/bear_grasp_task_node
  scripts/grasp_stack
  scripts/camera_masker_node
  scripts/camera_router_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
