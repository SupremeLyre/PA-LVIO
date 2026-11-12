cmake_minimum_required(VERSION 3.10)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    rosbag
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf
    pcl_conversions
    livox_ros_driver
)

add_definitions(-DROS_AVAILABLE=1)

catkin_package(
    LIBRARIES ${LVIO_LIBRARIES}
    CATKIN_DEPENDS message_runtime roscpp rosbag std_msgs sensor_msgs nav_msgs geometry_msgs tf pcl_conversions livox_ros_driver
)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/lvio
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_ros
    ROS/main.cc
    ROS/fusion.cc
    ROS/visual/visual_drawer_rviz.cc
    ROS/lidar/lidar_converter.cc
    ROS/lidar/lidar_viewer_rviz.cc
)
target_link_libraries(${PROJECT_NAME}_ros
    ${catkin_LIBRARIES}
    lvio
    stdc++fs
)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME}_ros ${PCL_LIBRARIES})

# abseil
find_package(absl REQUIRED)
target_link_libraries(${PROJECT_NAME}_ros
    absl::strings
    absl::time
)

install(TARGETS ${LVIO_LIBRARIES} ${PROJECT_NAME}_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
