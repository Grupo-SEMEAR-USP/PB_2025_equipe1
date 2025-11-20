cmake_minimum_required(VERSION 3.0.2)
project(robot_general)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package()

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)