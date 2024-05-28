import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("mecanum_robot")

    set_use_sim_time = SetParameter(name='use_sim_time', value=True)
    params_path = os.path.join(pkg_dir,'config','ball_tracker_params.yaml')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', [os.path.join(pkg_dir, 'config', 'ball_tracker.rviz')]]
    )

    tracker_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ball_tracker'), 'launch', 'ball_tracker.launch.py')]),
                    launch_arguments={'params_file': params_path,
                                    'image_topic': '/camera/image_raw',
                                    'cmd_vel_topic': '/cmd_vel',
                                    'enable_3d_tracker': 'true'}.items())


    return LaunchDescription([
        set_use_sim_time,
        rviz,
        tracker_launch,
    ])
