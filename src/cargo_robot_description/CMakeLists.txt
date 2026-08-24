cmake_minimum_required(VERSION 3.8)

project(cargo_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
