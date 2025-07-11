cmake_minimum_required(VERSION 3.5)
project(aruco_navigator)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  PROGRAMS
    aruco_navigator/aruco_nav.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
