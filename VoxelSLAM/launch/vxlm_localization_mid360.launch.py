from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    map_path = LaunchConfiguration('map_path')
    config = PathJoinSubstitution([
        FindPackageShare('voxel_slam'), 'config', 'ros2_localization_mid360.yaml'])
    rviz_config = PathJoinSubstitution([FindPackageShare('voxel_slam'), 'rviz_cfg', 'ros2.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('map_path', default_value=''),
        Node(package='voxel_slam', executable='voxelslam', name='voxelslam',
             output='screen', parameters=[config, {'Localization.map_path': map_path,
                                                    'General.mode': 'localization',
                                                    'General.map_export_path': ''}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_config], condition=IfCondition(rviz)),
    ])
