cmake_minimum_required(VERSION 3.8)
project(taskweave_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanStep.msg"
  "msg/Plan.msg"
  "msg/TaskStatus.msg"
  "msg/SkillInfo.msg"
  "srv/ListSkills.srv"
  "srv/StartMission.srv"
  "srv/TeachPoint.srv"
  "srv/DescribeRobot.srv"
  "action/RunSkill.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
