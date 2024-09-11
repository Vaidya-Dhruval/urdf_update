import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Declare path to the robot description (URDF) and RViz configuration file
        DeclareLaunchArgument('model', default_value=os.path.join(
            get_package_share_directory('combined_description'), 'urdf', 'combined_robot.urdf.xacro'), description='Path to URDF file'),
        DeclareLaunchArgument('rvizconfig', default_value=os.path.join(
            get_package_share_directory('combined_description'), 'rviz', 'view.rviz'), description='Path to RViz config file'),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),

        # Joint State Publisher Node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),

        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),


        # Static Transform Publisher Node for map -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        ),
    ])
