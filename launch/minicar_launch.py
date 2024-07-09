import os
import pathlib
from launch import LaunchDescription, substitutions, actions
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('minicar'),
        'config',
        'default.yaml'
    )

    logger = substitutions.LaunchConfiguration("log_level")
    return LaunchDescription([
        actions.DeclareLaunchArgument("log_level", default_value=["debug"], description="Logging level"),
        launch_ros.actions.Node(
            package='minicar',
            executable='localization',
            name='position',
            emulate_tty=True,
            parameters=[
                config
            ],
            arguments=['--ros-args', '--log-level', logger]
        ),
        launch_ros.actions.Node(
            package='minicar',
            executable='motorsManager',
            name='motors',
            emulate_tty=True,
            parameters=[
                config
            ],
            arguments=['--ros-args', '--log-level', logger]
        ),
        launch_ros.actions.Node(
            package='minicar',
            executable='accelerometer',
            name='accel',
            emulate_tty=True,
            parameters=[
                config
            ],
            arguments=['--ros-args', '--log-level', logger]
        ),
        launch_ros.actions.Node(
            package="minicar",
            executable="control",
            name="controller",
            emulate_tty=True,
            parameters=[config],
            arguments=['--ros-args', '--log-level', logger]
        ),
    ])