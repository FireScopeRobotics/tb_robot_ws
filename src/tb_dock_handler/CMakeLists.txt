cmake_minimum_required(VERSION 3.5)
project(tb_dock_handler)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


find_package(std_msgs REQUIRED)


install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/dock_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
