cmake_minimum_required(VERSION 2.8.3)
project(dwa_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            pcl_conversions
            collvoid_local_planner
            collvoid_srvs
            sensor_msgs
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
        )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

#link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/DWAPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES collvoid_melodic_dwa_local_planner
    CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
)

add_library(collvoid_melodic_dwa_local_planner
    src/dwa_planner.cpp 
    src/dwa_planner_ros.cpp
#    src/goal_alignment_cost_function.cpp
#    src/path_alignment_cost_function.cpp
#    include/collvoid_dwa_local_planner/
)
add_dependencies(collvoid_melodic_dwa_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(collvoid_melodic_dwa_local_planner collvoid_local_planner)
add_dependencies(collvoid_melodic_dwa_local_planner collvoid_scoring_function)
target_link_libraries(collvoid_melodic_dwa_local_planner base_local_planner ${catkin_LIBRARIES})

install(TARGETS collvoid_melodic_dwa_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
