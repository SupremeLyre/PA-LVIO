cmake_minimum_required(VERSION 3.10)

find_package(ament_cmake REQUIRED)
include(${CMAKE_CURRENT_LIST_DIR}/ROS2Compatibility.cmake)

find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_definitions(-DROS_AVAILABLE=2)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/lvio
)

set(ament_libraries
    rclcpp
    builtin_interfaces
    rosbag2_cpp
    rosbag2_storage
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    pcl_conversions
    livox_ros_driver2
)

add_executable(${PROJECT_NAME}_ros
    ROS/main.cc
    ROS/fusion.cc
    ROS/visual/visual_drawer_rviz.cc
    ROS/lidar/lidar_converter.cc
    ROS/lidar/lidar_viewer_rviz.cc
)
set_target_properties(${PROJECT_NAME}_ros PROPERTIES
    BUILD_WITH_INSTALL_RPATH FALSE
    INSTALL_RPATH "$ORIGIN/..:$ORIGIN${PA_LVIO_EXTRA_RPATH}"
)
ament_target_dependencies(${PROJECT_NAME}_ros ${ament_libraries})
target_link_libraries(${PROJECT_NAME}_ros
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

install(TARGETS ${LVIO_LIBRARIES}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_ros
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_export_libraries(${LVIO_LIBRARIES})
ament_export_dependencies(${ament_libraries})
ament_package()
