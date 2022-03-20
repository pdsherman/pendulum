cmake_minimum_required(VERSION 2.8.3)
project(pendulum)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_compile_options(
  $<$<COMPILE_LANGUAGE:C>:-std=c11>
  $<$<COMPILE_LANGUAGE:CXX>:-std=c++0x>
  $<$<COMPILE_LANGUAGE:CXX>:-lsqlite3>
  -Wall
  -fPIC
  -pthread
)

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -latomic")

# Find c++ threads library
find_package( Threads )

# Find ROS library
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

add_message_files( DIRECTORY ${PROJECT_SOURCE_DIR}/ros_msg
   FILES
   State.msg
   Control.msg
   Current.msg
   LoggingData.msg
)

add_service_files( DIRECTORY ${PROJECT_SOURCE_DIR}/ros_srv
  FILES
  DrawSystem.srv
  DeleteSystem.srv
  LoggingStart.srv
  LoggingDropTable.srv
  LoggingBufferCheck.srv
  LoggingStop.srv
  GraphData.srv
  MotorDriveMode.srv
  MotorControlGains.srv
  HardwareTest.srv
)

generate_messages( DEPENDENCIES std_msgs )

roslaunch_add_file_check(ros_launch)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

## Specify additional locations of header files
include_directories(${PROJECT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM "${PROJECT_SOURCE_DIR}/external-libs/libserial/src")

# Find SQLite3
find_package (SQLite3)
if (SQLITE3_FOUND)
  message("FOUND SQLITE")
  #include_directories(${SQLITE3_INCLUDE_DIRS})
endif (SQLITE3_FOUND)


# prefer static linking to shared objects
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_FIND_LIBRARY_SUFFIXES .a .so)

#set(BOOST_LIBRARYDIR "/usr/lib/arm-linux-gnueabihf/")
set(Boost_USE_STATIC_LIBS ON)
find_package(
    Boost 1.54 REQUIRED
    COMPONENTS atomic
               chrono
               system
               program_options)

add_subdirectory( tests )
add_subdirectory( hardware )
add_subdirectory( plant )
add_subdirectory( libs )
add_subdirectory( nodes )
add_subdirectory( external_libs )
