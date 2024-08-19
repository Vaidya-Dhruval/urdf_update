from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "description_pkg",
            description="The package where the robot description is located",
            default_value="cobotta_cell_description",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "description_file",
            description="The path to the robot description relative to the package root",
            default_value="urdf/cobotta_cell.urdf.xacro",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="jsp_gui",
            default_value="false",
            choices=["true", "false"],
            description="Flag to enable joint_state_publisher_gui",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            choices=["true", "false"],
            description="Whether to use RVIZ or not.",
        )
    )

    package_dir = FindPackageShare(LaunchConfiguration("description_pkg"))
    urdf_path = PathJoinSubstitution(
        [package_dir, LaunchConfiguration("description_file")]
    )

    robot_description_content = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
            }
        ],
    )

    ld.add_action(robot_state_publisher_node)

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("jsp_gui")),
        )
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([package_dir, "config", "dosplay.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[
            {
                "robot_description": robot_description_content,
            }
        ],
    )

    ld.add_action(rviz_node)

    return ld
