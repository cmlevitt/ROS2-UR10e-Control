from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ur_moveit_pkg = get_package_share_directory('ur_moveit_config')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_moveit_pkg, 'launch', 'ur_moveit.launch.py')
        ),
    launch_arguments={
        'ur_type': 'ur10e',
        'use_fake_hardware': 'false',
        'launch_rviz': 'false',
        'controllers_file': os.path.join(
            get_package_share_directory('UR_pick_n_place'),
            'config', 'ur_controllers.yaml'
        ),
    }.items(),
    )

    pnp_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='UR_pick_n_place',
                executable='pnp_routine',
                name='pnp_node',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        moveit_launch,
        pnp_node,
    ])