cmake_minimum_required(VERSION 3.5)
project(arduinobot_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()