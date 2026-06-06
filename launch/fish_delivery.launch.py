from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lift_height',     default_value='0.7'),
        DeclareLaunchArgument('safe_distance_m', default_value='0.5'),
        DeclareLaunchArgument('head_tilt_search',default_value='-0.8'),
        DeclareLaunchArgument('head_pan_search', default_value='0.0'),
        DeclareLaunchArgument('wait_sec',        default_value='30.0'),
        DeclareLaunchArgument('extend_target',   default_value='0.45'),
        DeclareLaunchArgument('extend_duration', default_value='20.0'),
        DeclareLaunchArgument('search_timeout',  default_value='15.0'),
        Node(
            package="fish_delivery",
            executable="fish_deliver",
            name="fish_deliverer",
            output="screen",
            parameters=[{
                'lift_height':      LaunchConfiguration('lift_height'),
                'safe_distance_m':  LaunchConfiguration('safe_distance_m'),
                'head_tilt_search': LaunchConfiguration('head_tilt_search'),
                'head_pan_search':  LaunchConfiguration('head_pan_search'),
                'wait_sec':         LaunchConfiguration('wait_sec'),
                'extend_target':    LaunchConfiguration('extend_target'),
                'extend_duration':  LaunchConfiguration('extend_duration'),
                'search_timeout':   LaunchConfiguration('search_timeout'),
            }],
        ),
    ])