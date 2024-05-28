import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import SetParameter

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_dir = get_package_share_directory("mecanum_robot")

    set_use_sim_time = SetParameter(name='use_sim_time', value=True)


    #using SLAM with params

    default_params_file = os.path.join(pkg_dir,
                                       'config', 'mapper_params_online_async.yaml')

    start_async_slam_toolbox_node = Node(
        parameters=[default_params_file],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', [os.path.join(pkg_dir, 'config', 'slam_mecanum_bot.rviz')]]
    )

    # Run the node
    return LaunchDescription([
        set_use_sim_time,
        start_async_slam_toolbox_node,
        rviz
    ])
