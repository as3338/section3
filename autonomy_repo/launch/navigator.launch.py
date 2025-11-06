#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 1. Declare 'use_sim_time' argument (Same as heading_control.launch.py)
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    # 3. Launch rviz.launch.py (Same as heading_control.launch.py)
    rviz_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py"]
        ),
        launch_arguments={
            "config": PathJoinSubstitution(
                [
                    FindPackageShare("autonomy_repo"),
                    "rviz",  # Assuming 'config' from PDF means 'rviz' folder
                    "default.rviz",
                ]
            ),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 2a. Launch rviz_goal_relay.py node
    # CHANGED: 'output_channel' is now '/cmd_nav'
    rviz_goal_relay_node = Node(
        executable="rviz_goal_relay.py",
        package="asl_tb3_lib",
        parameters=[
            {"output_channel": "/cmd_nav"},
        ],
    )

    # 2b. Launch state_publisher.py node (Same as heading_control.launch.py)
    state_publisher_node = Node(
        executable="state_publisher.py",
        package="asl_tb3_lib",
    )

    # 2c. Launch navigator.py node
    # CHANGED: 'executable' is 'navigator.py'
    # ADDED: 'use_sim_time' parameter
    navigator_node = Node(
        executable="navigator.py",
        package="autonomy_repo",
        parameters=[
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            rviz_launch,
            rviz_goal_relay_node,
            state_publisher_node,
            navigator_node,
        ]
    )