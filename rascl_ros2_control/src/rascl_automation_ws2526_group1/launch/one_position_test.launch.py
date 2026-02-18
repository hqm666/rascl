from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


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
    # - standalone process as ros2 Node
    # - has lifecycle, publishers, subscribers
    # - stays alive until it is killed

    automation_tsk2 = Node(
        package="rascl_automation_ws2526_group1",
        # namespace='turtlesim1',
        executable="one_position",
        # name='sim',
        output="screen",
    )

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

    pick_and_place_motion_controller = Node(
        package="rascl_automation_ws2526_group1",
        executable="pick_and_place_mc",
        name="pick_and_place_mc",
        output="both",
    )

    yasmin_node = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        output="log",
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

    # the motion controller and the automation task should start after initialization of the robot
    start_automation_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_spawner,
            on_exit=[pick_and_place_motion_controller, automation_tsk2],
        )
    )

    sigterm_timeout = LaunchConfiguration("sigterm_timeout")
    time_config_launch_arg = DeclareLaunchArgument(
        "sigterm_timeout", default_value="15"
    )

    return LaunchDescription(
        [
            # automation_tsk2,
            time_config_launch_arg,
            controller_manager_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_spawner,
            # joint_state_publisher_node,
            rviz_node,
            # pick_and_place_motion_controller,
            start_automation_after_controller,
            yasmin_node,
        ]
    )

