from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'raw_video',
        default_value='false',
        description='Set to true to enable raw (Y16) video mode'
    ))

    boson_node = Node(
        package="boson_ros2",
        executable="boson_node",
        name="boson_node",
        parameters=[{"raw_video": LaunchConfiguration("raw_video")}],
    )

    ld.add_action(boson_node)

    return ld
