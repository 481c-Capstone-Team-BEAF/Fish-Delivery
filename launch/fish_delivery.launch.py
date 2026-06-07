from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fish_delivery",
            executable="fish_deliver",
            name="fish_deliverer",
            output="screen",
        ),
    ])