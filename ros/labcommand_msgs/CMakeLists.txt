cmake_minimum_required(VERSION 3.8)
project(labcommand_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapBricks.msg"
  "msg/ChangeEvent.msg"
  "msg/MemoryStatus.msg"
  "msg/Activity.msg"
  "msg/Epoch.msg"
  "msg/RhythmMinute.msg"
  "msg/RequestSync.msg"
  "msg/OperatorAction.msg"
  "msg/SeqRange.msg"
  "msg/SyncEnd.msg"
  "srv/RequestSync.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
