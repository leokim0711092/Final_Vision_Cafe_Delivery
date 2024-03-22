import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="moveit_config_real")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/name.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_config_real") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            # moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

   

    return LaunchDescription(
        [
            rviz_node
        ]
 
    )

# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_moveit_rviz_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("name", package_name="moveit_config_real").to_moveit_configs()
#     return generate_moveit_rviz_launch(moveit_config)

