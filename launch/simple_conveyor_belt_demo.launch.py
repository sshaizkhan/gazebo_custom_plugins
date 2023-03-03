import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_ros_package_path = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_custom_plugins_package_path = FindPackageShare('gazebo_custom_plugins').find('gazebo_custom_plugins')

    world_file_path = os.path.join(gazebo_custom_plugins_package_path, 'worlds', 'box_spawn.world')
    urdf_file_path = os.path.join(gazebo_custom_plugins_package_path, 'urdf', 'simple_coveyor_belt.urdf')

    return LaunchDescription([
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [gazebo_ros_package_path, '/launch/gzclient.launch.py'])),
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [gazebo_ros_package_path, '/launch/gzserver.launch.py']),
                        launch_arguments={'world': world_file_path, 'verbose':'true'}.items()),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'coveyor_belt_demo', '-file', urdf_file_path],
            output='screen')
    ])
