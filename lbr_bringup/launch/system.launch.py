import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
    TimerAction
)
from launch.event_handlers import OnExecutionComplete


def generate_launch_description() -> LaunchDescription:
    
    lbr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("lbr_bringup"), "launch", "hardware.launch.py"
                )
            ]
        ),
        launch_arguments={"model": "med14"}.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("lbr_bringup"), "launch", "move_group.launch.py"
                )
            ]
        ),
        launch_arguments={"model": "med14", "mode": "hardware", "rviz": "false"}.items(),
    )

    pose_control_node = Node(
        package="lbr_demos_advanced_cpp",
        executable="pose_control",
        output="screen",
    )
    
    pose_planning_node = Node(
        package="lbr_demos_advanced_cpp",
        executable="pose_planning",
        output="screen",
    )

    ee_pose = Node(
        package="lbr_moveit_cpp",
        executable="get_ee_pose",
        output="screen",
    )

    hello_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("lbr_moveit_cpp"), "launch", "hello_moveit.launch.py"
                )
            ]
        ),
        launch_arguments={"model": "med14"}.items(),
    )

    return LaunchDescription(
        [
            lbr_bringup,
            move_group,
            pose_control_node,
            pose_planning_node,
            RegisterEventHandler(
                OnExecutionComplete(
                    target_action=move_group,
                    on_completion=[
                        LogInfo(msg='move group ready'),
                        TimerAction(
                            period=5.0,
                            actions=[hello_moveit, ee_pose],
                        )
                    ]
                )
            )
        ]
    )
