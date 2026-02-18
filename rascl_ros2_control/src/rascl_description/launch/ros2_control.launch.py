from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
)  # <--- hier richtig importieren
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # --- URDF/Xacro laden ---
    urdf_file = os.path.join(
        get_package_share_directory("rascl_description"), "urdf", "rascl.urdf"
    )
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    # --- Controller-Konfiguration laden ---
    controllers_yaml = os.path.join(
        get_package_share_directory("rascl_description"),
        "config",
        "controllers.yaml",
    )

    ############ Define Nodes #############
   
    # --- Controller Manager Node ---
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        output="screen",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )

   

    # --- Optional: Joint State Publisher GUI ---
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    # --- Optional: RViz ---
    rviz_config_file = os.path.join(
        get_package_share_directory("rascl_description"), "rviz", "urdf.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # output="screen",
        arguments=["-d", rviz_config_file],
    )

    ############# Spawners ##############
    # - temporary helper tool loading different controllers
    # - the set up controller runs inside an already existing ros2 control node
    # --- Joint State Broadcaster: reads the current joint states from the hwi and publishes them to /joint_states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    joint_trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
    )

    sigterm_timeout = LaunchConfiguration("sigterm_timeout")
    time_config_launch_arg = DeclareLaunchArgument(
        "sigterm_timeout", default_value="30"
    )

    return LaunchDescription(
        [
            time_config_launch_arg,
            controller_manager_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_spawner,
            # joint_state_publisher_node,
            rviz_node,
            # rqt_joint_traj_controller,
        ]
    )
