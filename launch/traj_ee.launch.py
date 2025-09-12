from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("svaya", package_name="moveit_config").to_moveit_configs()

    traj_node = Node(
        package='moveit_control',
        executable='traj_ee',
        name='pick_place',
        output='screen',
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {'use_sim_time': True},],
    )

    return LaunchDescription([ traj_node ])