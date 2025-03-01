import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'raw_video', default_value='false', description='Enable raw video mode'
        ),
        launch_ros.actions.Node(
            package='boson_ros2',
            executable='boson_node',
            name='boson_node',
            output='screen',
            parameters=[{'raw_video': LaunchConfiguration('raw_video')}]
        )
    ])