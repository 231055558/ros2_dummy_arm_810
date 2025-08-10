#!/usr/bin/env python3
"""
RViz控制Gazebo机械臂演示启动文件
此文件启动RViz和MoveIt，用于规划和控制Gazebo中的机械臂
注意：需要先启动gazebo_demo.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    生成RViz + MoveIt控制Gazebo机械臂的启动描述
    """

    # 使用Gazebo专用的MoveIt配置
    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
        .robot_description(file_path="config/dummy-ros2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers_gazebo.yaml")  # 使用Gazebo控制器配置
        .planning_pipelines()
        .to_moveit_configs()
    )

    # RViz配置文件
    rviz_config_file = os.path.join(
        get_package_share_directory("dummy_moveit_config"),
        "config",
        "moveit.rviz",
    )
    
    # RViz节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # 静态TF发布器 (world -> base_link)
    # 添加180度X轴旋转修正RViz中机械臂朝向与Gazebo一致
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "3.14159", "0.0", "0.0", "world", "base_link"],
    )

    # MoveGroup节点 - 用于运动规划
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},  # 使用仿真时间
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # MoveIt轨迹执行管理器
    moveit_controller_manager = Node(
        package="moveit_ros_move_group",
        executable="moveit_controller_manager",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # 注意：在Gazebo仿真中，D435相机数据由Gazebo插件提供
    # 不需要启动实际的realsense2_camera节点
    # 相机话题会由Gazebo的相机插件自动发布

    return LaunchDescription([
        static_tf,
        move_group_node,
        rviz_node,
    ]) 