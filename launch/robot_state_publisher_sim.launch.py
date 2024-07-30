import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def launch_setup(context, path: str, arm, *args, **kwargs):
    # Convert from type "LaunchConfiguration" to string
    arm_str = context.perform_substitution(arm)

    # Use xacro to process the file
    robot_description_raw = xacro.process_file(path, mappings={"arm_name": arm_str}).toxml()

    # Configure and return the robot_state_publisher node
    return  [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]
    )]

def generate_launch_description():
    # Obtain the path to xacro file
    pkg_name = 'robot_arm_visualization'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'robot_arm.urdf.xacro')

    # Obtain path to gazebo parameters file
    gazebo_params_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'gazebo_params.yaml')

    # Configure the execution for the gazebo launch file, to launch gazebo with the parameters file as an argument
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'extra_gazebo_arms': '--ros-args --params-file ' + gazebo_params_file}.items())

    # Configure node to spawn the robot arm from the description that was published to the robot_description topic by the robot_state_publisher node
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description','-entity', 'robot_arm'], output='screen')

    # Run the nodes
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value="arm_1"),
        gazebo,
        OpaqueFunction(function=launch_setup, args=[xacro_file, LaunchConfiguration('arm')]),
        spawn_entity
    ])