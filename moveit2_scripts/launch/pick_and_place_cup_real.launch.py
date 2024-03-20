import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="moveit_config_sim")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/name.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="pick_and_place_real_cup",
        package="moveit2_scripts",
        executable="pick_and_place_real_cup",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("moveit2_scripts"),
                "config",
                "arm_config.yaml",
            ),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription(
        [moveit_cpp_node],  
    )

