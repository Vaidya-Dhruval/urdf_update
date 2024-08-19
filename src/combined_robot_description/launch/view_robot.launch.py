from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the combined_robot_description package
    package_share_path = get_package_share_directory('combined_robot_description')

    return LaunchDescription([
        # Declare the RViz config file path argument
        DeclareLaunchArgument(
            'rviz_config',
            default_value='rviz/view_robot.rviz',
            description='Path to the RViz config file'
        ),

        # Declare the URDF/Xacro file path argument
        DeclareLaunchArgument(
            'urdf_file',
            default_value=[package_share_path, '/urdf/combined_robot_description.urdf.xacro'],
            description='Path to the URDF/Xacro file'
        ),

        # Launch robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')])}],
        ),
        
        # Launch joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        
        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        )
    ])
