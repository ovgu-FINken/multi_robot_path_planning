cmake_minimum_required(VERSION 2.8.3)
project(robot_path_costmap)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  pluginlib
  angles
  costmap_2d
  nav_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/NavigationPathLayer.cfg
)


catkin_package(
    CATKIN_DEPENDS angles costmap_2d dynamic_reconfigure geometry_msgs nav_msgs pluginlib roscpp
    INCLUDE_DIRS include
    LIBRARIES social_layers
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
    ${catkin_INCLUDE_DIRS}
)

## add cpp library
add_library(robot_path_costmap 
            src/navigation_paths.cpp
)


add_dependencies(robot_path_costmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(
    robot_path_costmap ${catkin_LIBRARIES}
)


if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS robot_path_costmap
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
