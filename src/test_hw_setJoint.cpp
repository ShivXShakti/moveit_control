#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <darm_msgs/msg/ui_command.hpp>
#include <darm_msgs/msg/ui_status.hpp>

using namespace std::chrono_literals;

class MoveItPlanner : public rclcpp::Node {
public:
    MoveItPlanner() : Node("dual_arm_planner") {
        traj_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("planned_trajectory", 10);
        pub_hw = this->create_publisher<darm_msgs::msg::UiCommand>("/svaya/ui/command", 10);
        sub_hw = this->create_subscription<darm_msgs::msg::UiStatus>("/svaya/ui/status", 10, 
            std::bind(&MoveItPlanner::callback, this, std::placeholders::_1)); 
    }

    void setupMoveGroups()
    {
        move_group_left_  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_left");
        move_group_right_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_right");
        move_group_both_  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_both");
        
        move_group_gripper_left_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), "gripper_left");
        move_group_gripper_right_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), "gripper_right");

        move_group_left_->setPlanningTime(10.0);
        move_group_right_->setPlanningTime(10.0);
        move_group_both_->setPlanningTime(10.0);
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Starting dual-arm trajectory planning…");
        /* ==============================
            1. MOVE TO GRASP POSE
           ==============================*/
        geometry_msgs::msg::Pose pose_left, pose_right;

        pose_left.orientation.x = 0.707;
        pose_left.orientation.y = 0.0;
        pose_left.orientation.z = 0.0;
        pose_left.orientation.w = 0.707;
        pose_left.position.x = 0.45;
        pose_left.position.y = 0.25;
        pose_left.position.z = 0.20;

        pose_right.orientation.x = 0.5;
        pose_right.orientation.y = -0.5;
        pose_right.orientation.z = 0.5;
        pose_right.orientation.w = -0.5;
        pose_right.position.x = 0.51;
        pose_right.position.y = -0.40;
        pose_right.position.z = -0.32;

        move_group_right_->setPoseTarget(pose_right);
        moveit::planning_interface::MoveGroupInterface::Plan plan_right;
        bool ok_r = (move_group_right_->plan(plan_right) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok_r) {
            RCLCPP_ERROR(this->get_logger(), "Right arm IK failed!");
            return;}
        auto right_joints = plan_right.trajectory_.joint_trajectory.points.back().positions;

        move_group_left_->setPoseTarget(pose_left);
        moveit::planning_interface::MoveGroupInterface::Plan plan_left;
        bool ok_l = (move_group_left_->plan(plan_left) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok_l) {
            RCLCPP_ERROR(this->get_logger(), "Left arm IK failed!");
            return;}
        auto left_joints = plan_left.trajectory_.joint_trajectory.points.back().positions;

        std::vector<double> full_joint_goal;
        full_joint_goal.insert(full_joint_goal.end(),
                               left_joints.begin(), left_joints.end());
        full_joint_goal.insert(full_joint_goal.end(),
                               right_joints.begin(), right_joints.end());
        move_group_both_->setJointValueTarget(full_joint_goal);

        moveit::planning_interface::MoveGroupInterface::Plan plan_both;
        bool ok_b = (move_group_both_->plan(plan_both) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (ok_b) {
            move_group_both_->execute(plan_both);
            execute_hw(plan_both);
            RCLCPP_INFO(this->get_logger(), "Both arms reached target poses!");}
        else{
            RCLCPP_ERROR(this->get_logger(), "Dual-arm plan failed!");
            return;}
    }

private:

    void execute_hw(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        rclcpp::Rate rate(10);  // 10 Hz streaming

        auto msg = darm_msgs::msg::UiCommand();
        msg.developer_command.enable = true;

        const auto &traj = plan.trajectory_.joint_trajectory;
        const size_t num_joints = traj.joint_names.size() + 2;

        msg.developer_command.command.resize(num_joints);

        for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i)
        {
            const auto &pt = traj.points[i];

            for (size_t j = 0; j < num_joints; ++j)
            {
                if (j <= 14){
                    msg.developer_command.command[j].position = pt.positions[j];
                    if (!pt.velocities.empty())
                        msg.developer_command.command[j].velocity = pt.velocities[j];
                    else
                        msg.developer_command.command[j].velocity = 0.0;
                }else{
                    msg.developer_command.command[j].position = 0.0;
                    msg.developer_command.command[j].velocity = 0.0;
                }
            }
            pub_hw->publish(msg);
            std::cout << "published joint " << i+1 << ": " << msg.developer_command.command[0].position << ", "<< msg.developer_command.command[1].position << ", "<< msg.developer_command.command[2].position << ", "<< msg.developer_command.command[3].position << ", "<< msg.developer_command.command[4].position << ", "<< msg.developer_command.command[5].position << ", "<< msg.developer_command.command[6].position << ", "
            << msg.developer_command.command[7].position << ", "<< msg.developer_command.command[8].position << ", "<< msg.developer_command.command[9].position << ", "<< msg.developer_command.command[10].position << ", "<< msg.developer_command.command[11].position << ", "<< msg.developer_command.command[12].position << ", "<< msg.developer_command.command[13].position << ", "<< msg.developer_command.command[14].position << ", "<<msg.developer_command.command[15].position << " rad" << std::endl;
            rate.sleep();
        }
    }

    void callback(darm_msgs::msg::UiStatus::SharedPtr msg){
        joint_msg = msg;
        joint_callback_status_ = true;
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_pub_;
    rclcpp::Subscription<darm_msgs::msg::UiStatus>::SharedPtr sub_hw;
    rclcpp::Publisher<darm_msgs::msg::UiCommand>::SharedPtr pub_hw;
    darm_msgs::msg::UiStatus::SharedPtr joint_msg;
    bool joint_callback_status_{false};

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_left_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_right_;
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
