from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Load the MoveIt configuration from the standard package
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "moveit.rviz",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    # Simple Trajectory Server (Bridge to Isaac Sim)
    trajectory_server_node = Node(
        package="simple_manipulation",
        executable="simple_trajectory_server",
        name="simple_trajectory_server",
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    # Static TF Publisher (World -> Panda Base)
    # Isaac Sim usually puts the robot at /World/Franka, but the URDF expects 'panda_link0'
    # We need a transform from 'world' to 'panda_link0'
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
        parameters=[{'use_sim_time': True}],
    )

    # Robot State Publisher
    # Publishes the TF tree based on the URDF and joint states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {'use_sim_time': True}],
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        trajectory_server_node,
        move_group_node,
        rviz_node,
    ])
