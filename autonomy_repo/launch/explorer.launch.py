#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 1. Declare 'use_sim_time' argument
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    # 3. Launch rviz.launch.py [cite: 177]
    rviz_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py"]
        ),
        launch_arguments={
            "config": PathJoinSubstitution(
                [
                    FindPackageShare("autonomy_repo"),
                    "rviz",
                    "default.rviz",
                ]
            ),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 2a. Launch rviz_goal_relay.py node
    rviz_goal_relay_node = Node(
        executable="rviz_goal_relay.py",
        package="asl_tb3_lib",
        parameters=[
            {"output_channel": "/cmd_nav"},
        ],
    )

    # 2b. Launch state_publisher.py node
    state_publisher_node = Node(
        executable="state_publisher.py",
        package="asl_tb3_lib",
    )

    # 2c. Launch navigator.py node (your existing navigator)
    navigator_node = Node(
        executable="navigator.py",
        package="autonomy_repo",
        parameters=[
            {"use_sim_time": use_sim_time}
        ]
    )

    # --- NEW ADDITION ---
    # 2d. Launch the new explorer.py node 
    explorer_node = Node(
        executable="explorer.py",
        package="autonomy_repo",
        parameters=[
            {"use_sim_time": use_sim_time}
        ],
        # output='screen', # Uncomment to see print statements
        # remappings=[ # Example of remapping if needed
        #     ('/map', '/my_map_topic')
        # ]
    )

    # Use a TimerAction to delay the explorer node's launch 
    # This gives other nodes (like SLAM and navigator) time to start up.
    delayed_explorer_node = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[explorer_node]
    )
    # --- END NEW ADDITION ---


    # This is the single launch file required 
    return LaunchDescription(
        [
            use_sim_time_arg,
            rviz_launch,
            rviz_goal_relay_node,
            state_publisher_node,
            navigator_node,
            
            # Add the delayed explorer node to the launch
            delayed_explorer_node,
        ]
    )