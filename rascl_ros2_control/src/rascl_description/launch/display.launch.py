import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    package_path = get_package_share_directory("rascl_description")
    urdf_path = os.path.join(package_path, "urdf", "rascl.urdf")
    rviz_config_path = os.path.join(package_path, "rviz", "urdf.rviz")

    # Read URDF from file
    with open(urdf_path, "r") as urdf_file:
        urdf = urdf_file.read()

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", rviz_config_path],
        )
    )

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": urdf}],
        )
    )

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher",
        )
    )

    return ld
