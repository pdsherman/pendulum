cmake_minimum_required(VERSION 2.8.3)
project(pendulum)

add_compile_options(
  $<$<COMPILE_LANGUAGE:C>:-std=c11>
  $<$<COMPILE_LANGUAGE:CXX>:-std=c++0x>
  -Wall
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  std_msgs
  message_generation
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files( DIRECTORY ${PROJECT_SOURCE_DIR}/msg
   FILES
   State.msg
   Control.msg
)

add_service_files( DIRECTORY ${PROJECT_SOURCE_DIR}/srv
  FILES
  AddPendulum.srv
  RemovePendulum.srv
)

generate_messages( DEPENDENCIES std_msgs )

roslaunch_add_file_check(launch)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

install( DIRECTORY gui/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS )

## Specify additional locations of header files
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory( tests )
#add_subdirectory( control )
add_subdirectory( devices )
add_subdirectory( plant )
