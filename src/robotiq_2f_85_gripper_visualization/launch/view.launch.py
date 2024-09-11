import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command([
                'xacro ', os.path.join(
                    get_package_share_directory('robotiq_2f_85_gripper_visualization'),
                    'urdf', 'robotiq_arg2f_85_model.xacro'
                )
            ])}]
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
            arguments=['-d', os.path.join(
                get_package_share_directory('robotiq_2f_85_gripper_visualization'),
                'visualize.rviz'
            )]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
