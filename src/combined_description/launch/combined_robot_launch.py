from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import FindPackageShare

def generate_launch_description():
    # Paths to the description and config files
    description_pkg_path = FindPackageShare("motoman_gp7_description").find("motoman_gp7_description")
    rviz_config_file = os.path.join(description_pkg_path, "config", "view.rviz")
    joint_names_config = os.path.join(description_pkg_path, "config", "joint_names_gp7.yaml")

    # Launch the robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=[os.path.join(description_pkg_path, "urdf", "gp7.urdf.xacro")]
    )

    # Launch the RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )

    # Launch the joint state publisher node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[joint_names_config]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_node,
    ])
