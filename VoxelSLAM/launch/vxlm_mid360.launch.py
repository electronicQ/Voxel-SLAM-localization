from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    config = PathJoinSubstitution([FindPackageShare('voxel_slam'), 'config', 'ros2_mid360.yaml'])
    rviz_config = PathJoinSubstitution([FindPackageShare('voxel_slam'), 'rviz_cfg', 'ros2.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='voxel_slam', executable='voxelslam', name='voxelslam',
             output='screen', parameters=[config]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_config], condition=IfCondition(rviz)),
    ])
