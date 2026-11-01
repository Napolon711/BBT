cmake_minimum_required(VERSION 3.8)
project(rm2027_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisState.msg"
  "msg/DecisionInput.msg"
  "msg/FieldEvent.msg"
  "msg/GameResult.msg"
  "msg/GameStatus.msg"
  "msg/GroundRobotPosition.msg"
  "msg/PowerHeat.msg"
  "msg/ProjectileAllowance.msg"
  "msg/RefereeFrame.msg"
  "msg/RefereeTx.msg"
  "msg/RefereeWarning.msg"
  "msg/RfidStatus.msg"
  "msg/RobotBuff.msg"
  "msg/RobotDamage.msg"
  "msg/RobotHP.msg"
  "msg/RobotPosition.msg"
  "msg/RobotStatus.msg"
  "msg/SentryCommand.msg"
  "msg/SentryInfo.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
