import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import xacro

def launch_setup(context, path: str, arm, *args, **kwargs):
    # Convert from type "LaunchConfiguration" to string
    arm_str = context.perform_substitution(arm)

    # Use xacro to process the file
    robot_description_raw = xacro.process_file(path, mappings={"arm_name": arm_str}).toxml()

    # Configure the node
    return  [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )]

def generate_launch_description():
    # Obtain the path to xacro file
    pkg_name = 'robot_arm_visualization'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'robot_arm.urdf.xacro')

    # Run the node
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value="arm_1"),
        OpaqueFunction(function=launch_setup, args=[xacro_file, LaunchConfiguration('arm')])
    ])