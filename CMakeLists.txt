cmake_minimum_required(VERSION 3.0.2)
project(mecanum_drive_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    urdf
    realtime_tools
    tf
    nav_msgs
    geometry_msgs
    control_msgs
    hardware_interface
    pluginlib)

find_package(Boost REQUIRED)

################################################
# List sources.
################################################
set(${PROJECT_NAME}_headers
  include/mecanum_drive_controller/mecanum_drive_controller.h
  include/mecanum_drive_controller/odometry.h
  include/mecanum_drive_controller/speed_limiter.h
)

set(${PROJECT_NAME}_sources
  src/mecanum_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    nav_msgs
    realtime_tools
    tf
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES mecanum_drive_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
