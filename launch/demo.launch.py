import os

# Core launch API
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

# ROS 2 nodes
from launch_ros.actions import Node

# MoveIt config utilities
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """
    This function contains all nodes and logic used to launch your
    Panda MoveIt2 setup with RViz2 and joint state broadcasting.
    """
    # Retrieve evaluated launch arguments from the context
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_external_rsp = LaunchConfiguration("use_external_rsp")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # ----------------------------------------------------------------------------
    # Build the MoveIt2 configuration using MoveItConfigsBuilder
    # This loads URDF, SRDF, kinematics.yaml, and planning pipeline settings.
    # It also includes moveit_controllers.yaml (for controller mapping).
    # ----------------------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_gazebo_moveit_config")
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=UnlessCondition(use_external_rsp)!= "true",  # Let MoveIt publish only if RSP not running externally
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # ----------------------------------------------------------------------------
    # Launch the main MoveIt move_group node
    # This handles motion planning and controller interfaces
    # ----------------------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time.perform(context) == "true"},
        ],
    )

    # ----------------------------------------------------------------------------
    # Conditionally launch robot_state_publisher
    # - If an external RSP is already running, don't start a new one
    # - Otherwise, start RSP with our generated robot_description
    # ----------------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time.perform(context) == "true"},
        ],
        condition=UnlessCondition(use_external_rsp),
    )

    # ----------------------------------------------------------------------------
    # Joint State Broadcaster spawner
    # This publishes joint_states so MoveIt can monitor robot state
    # Required even when using gz_ros2_control (since RViz2 listens on /joint_states)
    # ----------------------------------------------------------------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ----------------------------------------------------------------------------
    # Launch RViz2, pre-configured with MoveIt2 panels and robot view
    # Automatically loads robot_description and semantic config from MoveIt
    # ----------------------------------------------------------------------------
    rviz_config_file = os.path.join(
        get_package_share_directory("panda_gazebo_moveit_config"),
        "config",
        "urdf.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(launch_rviz),
    )

    # ----------------------------------------------------------------------------
    # Delay launching RViz until Joint State Broadcaster has started
    # This avoids RViz errors about missing TFs or joint_states
    # ----------------------------------------------------------------------------
    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # ----------------------------------------------------------------------------
    # Final list of nodes to start
    # ----------------------------------------------------------------------------
    return [
        robot_state_publisher_node,
        joint_state_broadcaster,
        delay_rviz_after_jsb,
        move_group_node,
    ]


def generate_launch_description():
    """
    This function defines and declares all launch arguments,
    and returns the complete LaunchDescription.
    """
    declared_arguments = []

    # Use simulation time (from Gazebo clock)
    declared_arguments.append(
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use sim time")
    )

    # Flag: If robot_state_publisher is running externally, don't start one here
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_external_rsp",
            default_value="false",
            description="Don't start internal RSP if one is running externally",
        )
    )

    # Flag: Launch RViz2
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz2 GUI?")
    )

    # Launch the system via OpaqueFunction (evaluates substitutions after parsing)
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


'''
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_gazebo_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
'''
