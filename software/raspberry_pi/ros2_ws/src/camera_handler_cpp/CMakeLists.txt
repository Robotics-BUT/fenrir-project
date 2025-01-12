cmake_minimum_required(VERSION 3.5)
project(camera_handler_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

add_executable(camera_handler_cpp src/main.cpp)
ament_target_dependencies(camera_handler_cpp rclcpp sensor_msgs cv_bridge OpenCV image_transport)

install(TARGETS
        camera_handler_cpp
        DESTINATION lib/${PROJECT_NAME})

ament_package()