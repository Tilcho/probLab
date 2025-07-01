from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Get full path to turtlebot3_world.launch.py
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_world_launch = os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')

    # Launch turtlebot3_world simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_world_launch)
    )

    # Kalman filter node
    kalman_node = Node(
        package='localization_filters',
        executable='kalman_filter_node',
        name='kalman_filter_node',
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        kalman_node,
        rviz_node
    ])