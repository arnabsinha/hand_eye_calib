cmake_minimum_required(VERSION 2.8.3)
project(hand_eye_calib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  tf
  geometry_msgs
  message_filters
  sensor_msgs
  pcl_ros
  url3Msg
)
find_package(message_generation)
find_package(url3Msg)
#add_message_files(FILES /home/ir/wp3/src/url3Msg/msg/url3Msg.msg)
catkin_package(CATKIN_DEPENDS message_runtime)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hand_eye_calib
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs tf geometry_msgs sensor_msgs message_filters pcl_ros url3Msg
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(hand_eye_calib
#   src/${PROJECT_NAME}/hand_eye_calib.cpp
# )

## Declare a cpp executable
add_executable(hand_eye_calib_node src/hand_eye_calib_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(hand_eye_calib_node hand_eye_calib_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hand_eye_calib_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS hand_eye_calib hand_eye_calib_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hand_eye_calib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
