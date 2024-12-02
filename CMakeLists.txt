cmake_minimum_required(VERSION 3.5)
project(turtlesim_connector)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_turtlesim_connector SHARED
  DIRECTORY src
)
target_include_directories(target_turtlesim_connector
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_turtlesim_connector
  PLUGIN "turtlesim_connector::TurtleSimConnector"
  EXECUTABLE turtlesim_connector_node
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
)