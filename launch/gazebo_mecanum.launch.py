import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import SetParameter

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'mecanum_robot'

    world_name = [os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                  'worlds', 'house.world')]

    set_use_sim_time = SetParameter(name='use_sim_time', value=True)

   # using old launch file to launch robot_state_publisher with mecanum bot

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'robot_state_publisher.launch.py'
        )]), launch_arguments={'use_sim_time':'True'}.items()
    )

    # including gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch',
            'gazebo.launch.py')]),
        launch_arguments={'world': world_name}.items()
    )

    #run a node for spawning mecanum bot
    spawn_entity = Node(package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description',
                   '-entity','mecanum_bot'],
        output='screen')



# CREATING CONTROLERS
    controller_params_file = os.path.join(get_package_share_directory(pkg_name),'config','controller_ros2.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file]
    )

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



    # run a node to convert /cmd_vel messages to mecanum wheels
    cont_mecanum = Node(
        package='mecanum_robot',
        executable='cont_mecanum.py',
    )

    map_odom = Node(
            package='mecanum_robot',
            executable='transform_odom.py',
    )



    #using SLAM with params

    default_params_file = os.path.join(get_package_share_directory("mecanum_robot"),
                                       'config', 'mapper_params_online_async.yaml')

    start_async_slam_toolbox_node = Node(
        parameters=[default_params_file],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')


    # Run the node
    return LaunchDescription([
        set_use_sim_time,
        rsp,
        gazebo,
        spawn_entity,

        controller_manager,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,

        cont_mecanum,
        map_odom,
        start_async_slam_toolbox_node

    ])

