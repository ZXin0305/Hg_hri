cmake_minimum_required(VERSION 3.0.2)
project(hg_hri)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG
"$ENV{CXXFLAGS}-O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE
"$ENV{CXXFLAGS} -O3 -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  visualization_msgs
  image_transport
  tf_conversions
  cv_bridge
  geometry_msgs

  kdl_parser
  trac_ik_lib
  byp80
  sensor_msgs 

  actionlib
  actionlib_msgs

  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS "OpenCV_version: ${OpenCV_VERSION}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HumanList.msg
  Human.msg
  PointCoors.msg
  robot2global.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  point2robot.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hg_hri
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib moveit_msgs actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  # src/HumanInfo.cpp
  # src/Gripper.cpp
  # src/Points.cpp
  # src/Robot.cpp
  # src/Detect.cpp

  src/HumanInfo.cpp
  src/Gripper.cpp
  src/fr3/FR3_points.cpp
  src/fr3/FR3_robot.cpp
  src/Detect.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/hg_hri_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# ====================================================================================
# ====================================================================================
# ====================================================================================

# 有关human的
add_executable(human_sub src/human_thread.cpp)
target_link_libraries(human_sub
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(detect_thread src/detect_thread.cpp)
target_link_libraries(detect_thread
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(fr3_robot src/fr3/FR3_robot.cpp)
target_link_libraries(fr3_robot
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# add_executable(main_fr3 src/fr3/main.cpp)
# target_link_libraries(main_fr3
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Eigen3_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )

add_executable(fr3_point src/fr3/FR3_points_thread.cpp)
target_link_libraries(fr3_point
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${OpenCV_LIBRARIES}
)


add_executable(record_trac src/fr3/record_trac.cpp)
target_link_libraries(record_trac
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${OpenCV_LIBRARIES}
)



# ====================================================================================
# ====================================================================================
# ====================================================================================


# add_executable(point_pub2 src/points_thread.cpp)
# target_link_libraries(point_pub2
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Eigen3_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )

# add_executable(robot_thread src/robot_thread.cpp)
# target_link_libraries(robot_thread
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Eigen3_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )

# # 测试可以运行，但是就是检测的断断续续的，不知道为什么，可能是录制的bag太卡了
# add_executable(detect_thread src/detect_thread.cpp)
# target_link_libraries(detect_thread
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Eigen3_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )

# add_executable(gripper_test src/Gripper.cpp)
# target_link_libraries(gripper_test
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Eigen3_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )

# add_executable(fr3_pose3 src/fr3/fr3_test.cpp)
# target_link_libraries(fr3_pose3
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Eigen3_LIBRARIES}
# )

# add_executable(fr3_attach src/fr3/fr3_test2.cpp)
# target_link_libraries(fr3_attach
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Eigen3_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )

# add_executable(fr3_test3 src/fr3/fr3_test3.cpp)
# target_link_libraries(fr3_test3
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Eigen3_LIBRARIES}
#     ${OpenCV_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hg_hri.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
