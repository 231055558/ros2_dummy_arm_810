#!/usr/bin/env python3
"""
Headless MoveIt Launch File for Real Robot without RViz
This launch file runs MoveIt without any GUI components for non-visual environments
while maintaining full planning scene and motion planning capabilities
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Use the ORIGINAL URDF configuration - this is critical for joint state sync
    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
        .robot_description(file_path="config/dummy-ros2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Static TF publisher - CRITICAL for coordinate frame transformations
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Robot State Publisher - CRITICAL for joint state visualization and tf
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # MoveGroup node for motion planning - THE CORE COMPONENT
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Planning Scene Monitor - Handles obstacle detection and planning scene updates
    # This is automatically started by move_group but we ensure it's configured properly
    
    print("🚀 启动无GUI的MoveIt系统...")
    print("📍 包含组件:")
    print("   ✅ MoveGroup 运动规划器")
    print("   ✅ Robot State Publisher")
    print("   ✅ Static TF Publisher") 
    print("   ✅ Planning Scene Monitor")
    print("   ❌ RViz (已禁用)")
    print("💡 障碍物脚本和规划器脚本仍可正常连接使用")

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            move_group_node,
        ]
    ) 