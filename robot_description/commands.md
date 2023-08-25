
## Best practice for `Motion_planning` implementation
In the following project


1. Rendered STL files are meshed in URDFS files. The meshed robot model can be viewed using this commands below and change the directory in command below.

   - ```
     ros2 launch urdf_tutorial display.launch.py model:=/home/ubuntu/ros2_project/src/robot_description/urdf/robot_des.urdf.xacro

     ```

2. The code is written in the Xacro format to create the Urdf of the robot model but ROS2 will accept only urdf format for model the robots. Lastly, it is must and should step to convert `xacro format to urdf formant` to publish the robot_description in the ROS2 node. This is command which helps to convert the files.

    ```
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/ubuntu/ros2_project/src/robot_description/urdf/robot_des.urdf.xacro)"

    ```
 - This GUI interface helps to control the joint states in ROS2
     ```
     ros2 run joint_state_publisher_gui joint_state_publisher_gui

     ```