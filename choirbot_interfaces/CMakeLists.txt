cmake_minimum_required(VERSION 3.5)
project(choirbot_interfaces)

# load requires packages
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate C++ and Python classes
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PositionAction.action"
  "msg/PositionTask.msg"
  "msg/PositionTaskArray.msg"
  "srv/PositionTaskService.srv"
  "srv/TaskCompletionService.srv"
)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(ament_cmake_python)

ament_package()
