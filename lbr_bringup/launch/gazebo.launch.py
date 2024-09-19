import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from lbr_bringup.description import LBRDescriptionMixin
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    AppendEnvironmentVariable,
    ExecuteProcess,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
import launch_ros.descriptions
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = os.path.join(get_package_share_directory("lbr_description"))
    xacro_file = os.path.join(pkg_path, "urdf", "med14", "med14.xacro")

    mode = "gazebo"

    robot_description_config = Command(["xacro ", xacro_file, " mode:=", mode])

    # Create a robot_state_publisher node
    params = {
        "robot_description": launch_ros.descriptions.ParameterValue(
            robot_description_config, value_type=str
        ),
        "use_sim_time": use_sim_time,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Set ignition resource path
    ign_resource_path = AppendEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.path.join(get_package_share_directory("lbr_description"), "meshes"),
    )

    bridge_params = os.path.join(
        get_package_share_directory("lbr_bringup"), "config", "ign_bridge.yaml"
    )

    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    world = os.path.join(
        get_package_share_directory("lbr_bringup"), "worlds", "empty_world.sdf"
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world]}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "med14",
            "-topic",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
        output="screen",
    )

    # controllers
    joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use sim time if true"
            ),
            LBRDescriptionMixin.arg_model(),
            LBRDescriptionMixin.arg_robot_name(),
            robot_state_publisher,
            ign_resource_path,
            gz_sim,
            start_gazebo_ros_bridge_cmd,
            spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_state_publisher,
                    on_exit=[spawn_entity],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[joint_trajectory_controller],
                )
            ),
        ]
    )
