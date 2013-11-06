cmake_minimum_required(VERSION 2.8.3)
project(syn)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  auv_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES syn
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${CATKIN_INCLUDE_DIRS})

## Declare a cpp library
# add_library(syn
#   src/${PROJECT_NAME}/syn.cpp
# )

## Declare a cpp executable
add_executable(vo src/vo.cpp)
add_executable(imu src/imu.cpp)
add_executable(q2p src/quaternion_to_rpy.cpp)

### For Testing starts ###
add_executable(tc1_imu unit_tests/tc1_imu.cpp)
add_executable(tc1_vo unit_tests/tc1_vo.cpp)

add_executable(tc2_imu unit_tests/tc2_imu.cpp)
add_executable(tc2_vo unit_tests/tc2_vo.cpp)

add_executable(tc3_vo unit_tests/tc3_vo.cpp)

add_executable(tc4_imu unit_tests/tc4_imu.cpp)

add_executable(tc5_imu unit_tests/tc5_imu.cpp)

add_executable(tc6_imu unit_tests/tc6_imu.cpp)
add_executable(tc6_vo unit_tests/tc6_vo.cpp)
### Testing ends ###

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(syn_node syn_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vo
  ${catkin_LIBRARIES}
)

target_link_libraries(imu
  ${catkin_LIBRARIES}
)

target_link_libraries(q2p
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############
target_link_libraries(tc1_vo
  ${catkin_LIBRARIES}
)

target_link_libraries(tc1_imu
  ${catkin_LIBRARIES}
)
target_link_libraries(tc2_vo
  ${catkin_LIBRARIES}
)

target_link_libraries(tc2_imu
  ${catkin_LIBRARIES}
)

target_link_libraries(tc3_vo
  ${catkin_LIBRARIES}
)

target_link_libraries(tc4_imu
  ${catkin_LIBRARIES}
)

target_link_libraries(tc5_imu
  ${catkin_LIBRARIES}
)

target_link_libraries(tc6_imu
  ${catkin_LIBRARIES}
)

target_link_libraries(tc6_vo
  ${catkin_LIBRARIES}
)
#Testing ends here


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
# install(TARGETS syn syn_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_syn.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
