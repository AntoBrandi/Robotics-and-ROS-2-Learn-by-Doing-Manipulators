cmake_minimum_required(VERSION 3.5)
project(arduinobot_description)


find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
