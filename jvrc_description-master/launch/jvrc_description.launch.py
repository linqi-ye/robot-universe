
# Standard library
import os

# External library
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """! Generate a launch description for the jvrc_description package."""
    urdf_file_name = 'jvrc1.urdf'
    urdf = os.path.join(
        get_package_share_directory('jvrc_description'),
        "urdf",
        urdf_file_name)
    with open(urdf, 'r') as file:
        robot_description = file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher
    ])