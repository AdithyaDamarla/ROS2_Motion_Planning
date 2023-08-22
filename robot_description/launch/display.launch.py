from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory("robot_description"), "urdf", "robot_des.urdf.xacro"),
        description="URDF file path location"
    )

    ros_des= ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_launch = Node(
           
        package ="robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": ros_des}]
    )

    joint_state_pub= Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    
    rviz2_launch= Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        robot_launch,
        joint_state_pub,
        rviz2_launch
    ])