from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    robot_description = ParameterValue(Command(['xacro ', os.path.join(get_package_share_directory("robot_description"),"urdf","robot_des.urdf.xacro")]),
                                       value_type=str)
 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', "--controller-manager","/controller_manager"]
    )
    
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', "--controller-manager","/controller_manager"]
    )
    
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', "--controller-manager","/controller_manager"]
    )
    

 
    return LaunchDescription([
        
        robot_state_publisher_node,
        joint_state_broadcaster_node,
        arm_controller_spawner,
        gripper_controller_spawner

    ])
