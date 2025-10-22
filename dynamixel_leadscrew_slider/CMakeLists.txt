cmake_minimum_required(VERSION 3.8)
project(dynamixel_leadscrew_slider)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# No interfaces defined in this runtime package; they are in dynamixel_leadscrew_slider_msgs
ament_export_dependencies(rosidl_default_runtime)
ament_package()


