from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from os import environ
from time import localtime, strftime


def generate_launch_description():
    homedir = environ['HOME']
    launchtime = strftime("%y%m%d-%H%M%S", localtime())
    return LaunchDescription([
        DeclareLaunchArgument('enable_window', default_value='False'),
        DeclareLaunchArgument('trajectory_file', default_value=f'{homedir}/trajectory-{launchtime}.txt'),
        DeclareLaunchArgument('keyframe_trajectory_file', default_value=f'{homedir}/keyframe-trajectory-{launchtime}.txt'),
        Node(
            package='ros2_vi_slam',
            executable='mono_vi_slam',
            namespace='ros2_vi_slam',
            name='mono_slam_sys',
            parameters = [
                { 
                 'enable_window': LaunchConfiguration('enable_window'),
                 'trajectory_file': LaunchConfiguration('trajectory_file'),
                 'keyframe_trajectory_file': LaunchConfiguration('keyframe_trajectory_file'),
                 },
                PathJoinSubstitution([
                    FindPackageShare('ros2_vi_slam'), 'config', 'topicconfigmono.yaml']),
                PathJoinSubstitution([
                    FindPackageShare('ros2_vi_slam'), 'config', 'fileconfigmono.yaml']),
            ],
        )
    ])
