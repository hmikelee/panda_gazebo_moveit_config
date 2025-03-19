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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    # Retrieve launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")      # .perform(context)
    gz_args = LaunchConfiguration("gz_args").perform(context)

    # Paths
    pkg_share = FindPackageShare("panda_gazebo_moveit_config").perform(context)
    xacro_file = os.path.join(pkg_share, "config", "panda.urdf.xacro")

    # ------------------------------------------------------------------------
    # Robot Description (evaluated inline using xacro)
    # ------------------------------------------------------------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ])
    print(robot_description_content)

    robot_description = {"robot_description": robot_description_content.perform(context)}

    # ------------------------------------------------------------------------
    # Launch Gazebo Sim (Harmonic) via ros_gz_sim standard launch file
    # ------------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": gz_args}.items()
    )

    # ------------------------------------------------------------------------
    # Spawn Panda robot into Gazebo using ros_gz_sim create
    # ------------------------------------------------------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description["robot_description"],
            "-name", "panda",
            "-allow_renaming", "true"
        ]
    )

    # ------------------------------------------------------------------------
    # robot_state_publisher for TF broadcasting
    # ------------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ]
    )
    

    # ------------------------------------------------------------------------
    # Clock bridge: Gazebo → ROS 2 time sync
    # ------------------------------------------------------------------------
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    

    # ------------------------------------------------------------------------
    # Controller spawners for ros2_control
    # Controllers must be defined in ros2_controllers.yaml
    # ------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return [
        gz_sim,
        clock_bridge,
        robot_state_publisher_node,
        spawn_robot,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        hand_controller_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        # Launch argument: Gazebo startup arguments
        DeclareLaunchArgument(
            "gz_args",
            default_value='-r -v4 empty.sdf',
            description="Arguments to pass to gz sim (e.g., -r -v4 world.sdf)"
        ),

        # Launch argument: Enable simulated time in ROS 2
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time from /clock topic"
        ),

        # Launch actions (in order)
        OpaqueFunction(function=launch_setup)
    ])
