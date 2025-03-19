#!/usr/bin/env python3
# MIT License
# Copyright (c) 2025 Mike Lee (hmikelee@gmail.com)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # ----------------------------------------------------------------------------
    # 1. Generate MoveIt2 Configuration using MoveItConfigsBuilder
    # ----------------------------------------------------------------------------
    moveit_config = MoveItConfigsBuilder(
        robot_name="panda",
        package_name="panda_gazebo_moveit_config"
    ).to_moveit_configs()

    # ----------------------------------------------------------------------------
    # 2. Path to RViz2 Configuration File
    #     - Loads pre-defined panels, Planning tab, robot model, etc.
    # ----------------------------------------------------------------------------
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("panda_gazebo_moveit_config"),
        "config",
        "urdf.rviz"
    ])

    # ----------------------------------------------------------------------------
    # 3. Launch Move Group Node (core planning backend of MoveIt2)
    # ----------------------------------------------------------------------------
    move_group_node = generate_move_group_launch(moveit_config)

    # ----------------------------------------------------------------------------
    # 4. Launch RViz2 Node with MoveIt2 plugins and robot model
    # ----------------------------------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": True},  # 🔧 Ensures RViz uses /clock from simulation
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
        ],
    )

    # ----------------------------------------------------------------------------
    # 5. Return Combined Launch Description
    # ----------------------------------------------------------------------------
    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])

