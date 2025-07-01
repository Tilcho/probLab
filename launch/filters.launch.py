import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # === Base paths ===
    workspace_root = os.path.dirname(os.path.dirname(__file__))
    config_dir = os.path.join(workspace_root, 'config')

    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # === Config files ===
    waypoint_config = os.path.join(config_dir, 'waypoints.yaml')
    nav2_params = os.path.join(config_dir, 'nav2_params.yaml')
    map_file = os.path.join(config_dir, 'map_1749886992.yaml')

    # === Launch args ===
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    world = os.path.join(tb3_gazebo, 'worlds', 'turtlebot3_world.world')

    # === Gazebo ===
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo, 'models')
    )

    # === Nav2 ===
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'map': map_file,
            'params_file': nav2_params
        }.items()
    )

    # === Custom nodes ===
    navigator_node = Node(
        package='localization_filters',
        executable='navigator',
        name='navigator',
        output='screen',
        parameters=[waypoint_config]
    )

    # Delay navigator until Nav2 stack is likely ready
    delayed_navigator = TimerAction(
        period=5.0,
        actions=[navigator_node]
    )

    kf_node = Node(
        package='localization_filters',
        executable='kf_node',
        name='kf_node',
        output='screen'
    )

    ekf_node = Node(
        package='localization_filters',
        executable='ekf_node',
        name='ekf_node',
        output='screen'
    )

    pf_node = Node(
        package='localization_filters',
        executable='pf_node',
        name='pf_node',
        output='screen'
    )

    # === Launch description ===
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(set_env_vars_resources)

    ld.add_action(nav2)
    ld.add_action(kf_node)
    ld.add_action(ekf_node)
    ld.add_action(pf_node)
    ld.add_action(delayed_navigator)

    return ld
