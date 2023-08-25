from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory("robot_description"), "urdf", "robot_des.urdf.xacro"),
        description="URDF file path location"
    )

    env_var=SetEnvironmentVariable("GAZEBO_MOBEL_PATH", os.path.join(get_package_prefix("robot_description"), "share"))
    ros_des= ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_launch = Node(
           
        package ="robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": ros_des}]
    )

    start_gazebo_server =IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"))
    )
    start_gazebo_client= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch","gzclient.launch.py"))
    )

    spawn_robot= Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "arm_robot", "-topic", "robot_description"],
        output="screen"
    )
    
    return LaunchDescription([
        model_arg,
        robot_launch,
        env_var,
        start_gazebo_client,
        start_gazebo_server,
        spawn_robot
    ])