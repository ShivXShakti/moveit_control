#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std::chrono_literals;

class MoveItPlanner : public rclcpp::Node {
public:
    MoveItPlanner() : Node("dual_arm_planner") 
    {
        traj_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "planned_trajectory", 10);
    }

    void setupMoveGroups()
    {
        move_group_left_  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_left");

        move_group_right_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_right");

        move_group_both_  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_both");

        move_group_left_->setPlanningTime(10.0);
        move_group_right_->setPlanningTime(10.0);
        move_group_both_->setPlanningTime(10.0);
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Starting dual-arm trajectory planning…");

        // ------------------------------
        // 1. Define target poses
        // ------------------------------
        geometry_msgs::msg::Pose pose_left, pose_right;

        pose_left.orientation.x = 0.707;
        pose_left.orientation.y = 0.0;
        pose_left.orientation.z = 0.0;
        pose_left.orientation.w = 0.707;
        pose_left.position.x = 0.45;
        pose_left.position.y = 0.20;
        pose_left.position.z = 0.20;

        pose_right.orientation.x = -0.707;
        pose_right.orientation.y = 0.0;
        pose_right.orientation.z = 0.0;
        pose_right.orientation.w = 0.707;
        pose_right.position.x = 0.45;
        pose_right.position.y = -0.20;
        pose_right.position.z = -0.10;

        // ------------------------------
        // 2. IK for right arm
        // ------------------------------
        move_group_right_->setPoseTarget(pose_right);

        moveit::planning_interface::MoveGroupInterface::Plan plan_right;
        bool ok_r = (move_group_right_->plan(plan_right) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

        if (!ok_r) {
            RCLCPP_ERROR(this->get_logger(), "Right arm IK failed!");
            return;
        }

        auto right_joints =
            plan_right.trajectory_.joint_trajectory.points.back().positions;

        // ------------------------------
        // 3. IK for left arm
        // ------------------------------
        move_group_left_->setPoseTarget(pose_left);

        moveit::planning_interface::MoveGroupInterface::Plan plan_left;
        bool ok_l = (move_group_left_->plan(plan_left) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

        if (!ok_l) {
            RCLCPP_ERROR(this->get_logger(), "Left arm IK failed!");
            return;
        }

        auto left_joints =
            plan_left.trajectory_.joint_trajectory.points.back().positions;

        // ------------------------------
        // 4. Combine joint values for both arms
        // ORDER MUST MATCH SRDF FOR arm_both !
        // ------------------------------
        std::vector<double> full_joint_goal;
        full_joint_goal.insert(full_joint_goal.end(),
                               left_joints.begin(), left_joints.end());
        full_joint_goal.insert(full_joint_goal.end(),
                               right_joints.begin(), right_joints.end());

        move_group_both_->setJointValueTarget(full_joint_goal);

        // ------------------------------
        // 5. Whole-body plan
        // ------------------------------
        moveit::planning_interface::MoveGroupInterface::Plan plan_both;
        bool ok_b = (move_group_both_->plan(plan_both) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

        if (!ok_b) {
            RCLCPP_ERROR(this->get_logger(), "Dual-arm plan failed!");
            return;
        }

        move_group_both_->execute(plan_both);
        RCLCPP_INFO(this->get_logger(), "Both arms reached target poses!");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_pub_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_left_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_right_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_both_;
};

// ------------------ MAIN ------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MoveItPlanner>();
    node->setupMoveGroups();
    node->run();

    rclcpp::shutdown();
    return 0;
}
