from typing import List, Optional, Union

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class GazeboMixin:
    @staticmethod
    def include_gazebo(**kwargs) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ),
            launch_arguments={'gz_args': ['-r ', 'empty.sdf'], 'on_exit_shutdown': 'true'}.items(),
            **kwargs,
        )

    @staticmethod
    def node_spawn_entity(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        tf: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        **kwargs,
    ) -> Node:
        label = ["-x", "-y", "-z", "-R", "-P", "-Y"]
        tf = [str(x) for x in tf]
        return Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                robot_name,
                "-topic",
                "robot_description",
            ]
            + [item for pair in zip(label, tf) for item in pair],
            output="screen",
            namespace=robot_name,
            **kwargs,
        )
