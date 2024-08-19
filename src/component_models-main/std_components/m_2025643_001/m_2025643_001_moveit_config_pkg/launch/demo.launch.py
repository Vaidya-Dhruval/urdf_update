import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
import yaml


def load_file(package_dir, file_path):
    absolute_file_path = os.path.join(package_dir, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        raise EnvironmentError(f'Did not find {absolute_file_path}')


def load_yaml(package_dir, file_path):
    absolute_file_path = os.path.join(package_dir, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        raise EnvironmentError(f'Did not find {absolute_file_path}')


def nested_set(dic, keys, value):
    for key in keys[:-1]:
        dic = dic.setdefault(key, {})
    dic[keys[-1]] = value


def nested_get(dic, keys):
    for key in keys[:-1]:
        dic = dic.get(key)
    print(dic)
    print(keys[-1])
    return dic.get(keys[-1])


def generate_launch_description():
    ld = LaunchDescription()

    moveit_dir = get_package_share_directory('m_2025643_001_moveit_config_pkg')

    use_fake_controller = LaunchConfiguration('use_fake_controller')

    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value="False",
        description='if start rviz2')

    ld.add_action(declare_use_rviz_cmd)

    declare_use_fake_controller_cmd = DeclareLaunchArgument(
        'use_fake_controller',
        default_value="True",
        description='Use fake controller')
    ld.add_action(declare_use_fake_controller_cmd)

    # planning context
    robot_description_file = os.path.join(
        moveit_dir, 'config/m_2025643_001.urdf.xacro')
    robot_description_semantic_config = load_file(
        moveit_dir, 'config/m_2025643_001.srdf')
    kinematics_yaml = load_yaml(moveit_dir, 'config/kinematics.yaml')
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_yaml}
    # sensors_yaml = load_yaml(moveit_dir, 'config/sensors_3d.yaml')

    robot_description = {'robot_description': Command(
        ['xacro ', robot_description_file])}
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    joint_limits_yaml = load_yaml(moveit_dir, 'config/joint_limits.yaml')
    robot_description_planning = {
        "robot_description_planning": {
            **joint_limits_yaml
        }
    }

    # Planning Functionality
    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["pilz", "ompl"],
        "pilz": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "",
            "start_state_max_bounds_error": 0.1,
        },
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_dir, "config/ompl_planning.yaml"
                                   )
    planning_pipelines_config["ompl"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        moveit_dir, "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )
    ld.add_action(run_move_group_node)

    # RViz
    rviz_config_file = os.path.join(moveit_dir, 'config', 'moveit.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     condition=IfCondition(use_rviz),
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 robot_description_planning,
                                 robot_description_kinematics,
                                 planning_pipelines_config, ])

    ld.add_action(rviz_node)

    # static_tf = Node(package='tf2_ros',
    #                  executable='static_transform_publisher',
    #                  name='static_transform_publisher',
    #                  output='log',
    #                  arguments=['0.0', '0.0', '0.0', '0.0',
    #                             '0.0', '0.0', 'world', 'base_link'],
    #                  condition=IfCondition(use_fake_controller))
    # ld.add_action(static_tf)

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description],
                                 condition=IfCondition(use_fake_controller))
    ld.add_action(robot_state_publisher)

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(moveit_dir,
                                         "config",
                                         "ros2_controllers.yaml",
                                         )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )
    ld.add_action(ros2_control_node)

    for controller in ['m_2025643_001_controller',
                       'joint_state_broadcaster']:
        ld.add_action(
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(
                    controller)],
                shell=True,
                output='screen',
                condition=IfCondition(use_fake_controller)
            )
        )

    return ld
