cmake_minimum_required(VERSION 3.0.2)
project(robot_vision)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  robot_communication
  #numpy
  #cv2
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    robot_communication
    #numpy
    #cv2
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Adding Config Files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)