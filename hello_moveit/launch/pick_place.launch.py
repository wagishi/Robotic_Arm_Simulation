from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "gen3",
            package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config",
        )
        .to_moveit_configs()
    )

    pick_place_node = Node(
        package="hello_moveit",
        executable="pick_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([pick_place_node])
