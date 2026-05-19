cmake_minimum_required(VERSION 3.8)
project(controllers_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianDeltaPoint.msg"
  "msg/CartesianDeltaState.msg"
  "msg/TcpRotvecDeltaPoint.msg"
  "msg/TcpRotvecDeltaState.msg"
  "action/FollowCartesianDelta.action"
  "action/FollowTcpRotvecDelta.action"
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
