import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("darm_tmv", package_name="moveit_config_tmv").to_moveit_configs()
    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="pick_place",
        package="moveit_control",
        executable="cam_pick_place_setPose",
        output="screen",
        parameters=[
            moveit_config.robot_description, 
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )
    pose_publisher = Node(
        name="object_pose_transformer",
        package="robot_perception",
        executable="pose_publisher",
        output="screen"
    )

    # yolo_oakd = IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource([PathJoinSubstitution(
    #                     [FindPackageShare('depthai_examples'), 'launch', 'tracker_yolo_spatial_node.launch.py'])]),

    #             )

    return LaunchDescription(
    [moveit_cpp_node,
     pose_publisher,
     #yolo_oakd,
     ]
    )

 