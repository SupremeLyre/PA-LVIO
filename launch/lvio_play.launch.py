import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pa_lvio")
    default_config = os.path.join(pkg_share, "config", "lvio.yaml")
    default_rviz = os.path.join(pkg_share, "config", "lvio-rviz2.rviz")

    configfile_arg = DeclareLaunchArgument(
        "configfile",
        default_value=default_config,
        description="Path to the PA-LVIO YAML configuration file",
    )
    bagfile_arg = DeclareLaunchArgument(
        "bagfile",
        default_value="",
        description="Path to the ROS 2 bag directory/file to play",
    )
    rviz_enable_arg = DeclareLaunchArgument(
        "rviz_enable",
        default_value="true",
        description="Launch RViz2 visualization",
    )

    rosbag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", LaunchConfiguration("bagfile")],
        output="log",
    )

    lvio_node = Node(
        package="pa_lvio",
        executable="pa_lvio_ros",
        name="lvio_node",
        output="screen",
        arguments=[LaunchConfiguration("configfile")],
        parameters=[
            {"is_read_bag": False},
        ],
    )

    tf_world_map_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_map",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--yaw", "0",
            "--pitch", "0",
            "--roll", "3.141592653589793",
            "--frame-id", "world",
            "--child-frame-id", "map",
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=["-d", default_rviz],
    )

    return LaunchDescription(
        [
            configfile_arg,
            bagfile_arg,
            rviz_enable_arg,
            rosbag_play,
            lvio_node,
            tf_world_map_node,
            rviz_node,
        ]
    )
