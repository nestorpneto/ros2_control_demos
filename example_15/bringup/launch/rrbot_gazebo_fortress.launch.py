import os
import xacro

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ros2_control_demo_example_15_path = os.path.join(
        get_package_share_directory('ros2_control_demo_example_15'))

    ros_gz_sim_path = os.path.join(
        get_package_share_directory('ros_gz_sim'))

    # World file name
    world_file = os.path.join(ros2_control_demo_example_15_path, 'worlds', 'floor.sdf')


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_demo_example_15"), "urdf", "rrbot.urdf.xacro"]
            ),
            " ",
            "use_gazebo:=true",
            " ",
            "gazebo_fortress:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_content,
                   '-name', 'rrbot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.0',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-allow_renaming', 'true'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            'force_torque@geometry_msgs/msg/Wrench[ignition.msgs.Wrench'
        ],
        output='screen'
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r {}'.format(world_file)}.items(),
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
        gz_launch,
        node_robot_state_publisher,
        bridge,
        ignition_spawn_entity,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ])
