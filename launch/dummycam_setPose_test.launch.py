import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("darm_tmv", package_name="moveit_config_tmv").to_moveit_configs()
    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="pick_place",
        package="moveit_control",
        executable="dummycam_setPose_traj_test",
        output="screen",
        parameters=[
            moveit_config.robot_description, 
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )
    pose_node = Node(
        name="object_pose_transformer",
        package="robot_perception",
        executable="dummy_pose",
        output="screen",
    )

    return LaunchDescription(
    [moveit_cpp_node,
     pose_node
     ]
    )

 