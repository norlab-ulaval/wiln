import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration('wiln_ns')
    namespace_launch_arg = DeclareLaunchArgument(
        'wiln_ns',
        default_value='wiln'
    )

    share_folder = get_package_share_directory('norlab_robot')
    config_file = os.path.join(share_folder, "config", "_wiln.yaml")

    wiln_node = Node(
        package='wiln',
        executable='wiln_node',
        name='wiln_node',
        namespace=namespace,
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        namespace_launch_arg,
        wiln_node
    ])