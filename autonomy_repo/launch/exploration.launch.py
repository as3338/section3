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

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    # 1. Launch RViz (Same as provided file)
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

    # 2. Launch rviz_goal_relay.py (Helper for manual clicking in RViz if needed)
    rviz_goal_relay_node = Node(
        executable="rviz_goal_relay.py",
        package="asl_tb3_lib",
        parameters=[
            {"output_channel": "/cmd_nav"},
        ],
    )

    # 3. Launch state_publisher.py (Required for /state topic)
    state_publisher_node = Node(
        executable="state_publisher.py",
        package="asl_tb3_lib",
    )

    # 4. Launch navigator.py (Handles path planning and control)
    navigator_node = Node(
        executable="navigator.py",
        package="autonomy_repo",
        parameters=[
            {"use_sim_time": use_sim_time}
        ]
    )

    # 5. Launch frontier_explorer.py (YOUR NEW NODE)
    # This node implements the exploration logic 
    # Assumes the node script is executable and in the 'autonomy_repo' package
    frontier_explorer_node = Node(
        executable="frontier_explorer.py",
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
            frontier_explorer_node, # Added to launch description
        ]
    )