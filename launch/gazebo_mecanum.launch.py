import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'mecanum_robot'


   # using old launch file to launch robot_state_publisher with mecanum bot

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'robot_state_publisher.launch.py'
        )]), launch_arguments={'use_sim_time':'true'}.items()
    )

    # including gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch',
            'gazebo.launch.py')]),
    )

    #run a node for spawning mecanum bot
    spawn_entity = Node(package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description',
                   '-entity','mecanum_bot'],
        output='screen')


# CREATING CONTROLERS

    joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

    robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["right_front_controller",
                       "left_front_controller",
                       "right_rear_controller",
                       "left_rear_controller"],
        )




    # controll_spawn = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', 'right_front_controller',
    #                'left_front_controller', 'right_rear_controller',
    #                'left_rear_controller', "--controller-manager", "/controller_manager"],
    # )






    # joint_state_broadcaster_spawner = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #     )
    #
    # robot_controller_spawner = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["diff_cont", "--controller-manager", "/controller_manager"],
    #     )


    # Run the node
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,


        joint_state_broadcaster_spawner,
        robot_controller_spawner,

        # diff_drive_spawner,
        # joint_broad_spawner
        # controll_spawn
    ])

