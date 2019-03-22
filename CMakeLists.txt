cmake_minimum_required(VERSION 2.8.3)
project(mpc_path_follower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
include_directories(/usr/include)
link_directories(/usr/lib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_path_follower
  CATKIN_DEPENDS base_local_planner costmap_2d nav_core nav_msgs pluginlib roscpp tf
#  DEPENDS system_lib
)

## Declare a C++ library
add_library(mpc_path_follower
 src/mpc_path_follower_ros.cpp
 )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
target_link_libraries(${PROJECT_NAME} ipopt)

#add_dependencies(mpc_path_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS mpc_path_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES blp_mpc_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


