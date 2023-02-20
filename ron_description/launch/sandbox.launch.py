from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join as ojoin

def generate_launch_description():
    desc = LaunchDescription()
    desc.add_action(Node(
        package="ron_description",
        executable="ron_configuration",
        namespace="/ron",
        parameters=[ojoin(
            get_package_share_directory("ron_description"),
            "config",
            "sandbox.yaml"
        )],
        on_exit=Shutdown(),
    ))
    return desc
