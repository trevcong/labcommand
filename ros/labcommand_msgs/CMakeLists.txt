cmake_minimum_required(VERSION 3.8)
project(labcommand_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapBricks.msg"
  "msg/ChangeEvent.msg"
  "msg/MemoryStatus.msg"
  "msg/Epoch.msg"
  "msg/RequestSync.msg"
  "msg/Activity.msg"
  "msg/OperatorAction.msg"
  "msg/SeqRange.msg"
  "msg/SyncEnd.msg"
  "msg/RigPose.msg"
  "msg/ObjectLink.msg"
  "msg/RecoveredAccept.msg"
  "msg/NameableSeed.msg"
  "msg/RegistryObject.msg"
  "msg/ObjectRegistry.msg"
  "srv/RequestSync.srv"
  "srv/NameObject.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
