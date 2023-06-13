cmake_minimum_required(VERSION 3.0.2)
project(onrobot_mg_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
  onrobot_mg_modbus_tcp
  message_generation
)

add_message_files(
  FILES
  OnRobotMGInput.msg
  OnRobotMGOutput.msg
) 

add_service_files(
  FILES
  SetCommand.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
  CATKIN_DEPENDS
  rospy
  onrobot_mg_modbus_tcp
)

catkin_install_python(
  PROGRAMS
  nodes/OnRobotMGSimpleController.py
  nodes/OnRobotMGStatusListener.py
  nodes/OnRobotMGTcpNode.py
  nodes/OnRobotMGSimpleControllerServer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
