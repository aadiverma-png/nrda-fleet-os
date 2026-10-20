cmake_minimum_required(VERSION 3.8)
project(nrda_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/Claim.msg"
  "msg/ClaimSet.msg"
  "msg/Bid.msg"
  "msg/Task.msg"
  "msg/TaskEvent.msg"
  "msg/Alert.msg"
  "msg/Control.msg"
  "msg/SafetyEvent.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleList.msg"
)
ament_package()
