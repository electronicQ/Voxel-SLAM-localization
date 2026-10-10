from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('voxel_slam'), 'config', 'ros2_scan_planner_adapter.yaml'])
    return LaunchDescription([
        Node(package='voxel_slam', executable='scan_planner_odom_adapter',
             name='scan_planner_odom_adapter', output='screen', parameters=[config]),
    ])
