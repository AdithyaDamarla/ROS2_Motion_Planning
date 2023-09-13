#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>

void move_arm_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group= moveit::planning_interface::MoveGroupInterface(node, "arm");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    std::vector<double> arm_joint_movement {1.57, 0.0, 0.0};
    std::vector<double>  gripper_joint_movement {-0.7, 0.7};

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_movement);
    bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_movement);

    if (!arm_within_bounds | !gripper_within_bounds){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint position were outside the limits");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

    bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit:: core:: MoveItErrorCode::SUCCESS;
    bool gripper_plan_success = arm_move_group.plan(gripper_plan) == moveit:: core:: MoveItErrorCode::SUCCESS;

    if (arm_plan_success && gripper_plan_success){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner succeed, moving the arm and the gripper");
        arm_move_group.move();
        gripper_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "one or more planners failed");
    }

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    std:: shared_ptr<rclcpp::Node> node = rclcpp:: Node:: make_shared("arm_robot_moveit_interface");

    move_arm_robot(node);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}