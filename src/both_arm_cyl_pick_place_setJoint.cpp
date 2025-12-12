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
    MoveItPlanner() : Node("dual_arm_planner") {}

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
        if (!ok_b) {
            RCLCPP_ERROR(this->get_logger(), "Dual-arm plan failed!");
            return;}
        move_group_both_->execute(plan_both);
        RCLCPP_INFO(this->get_logger(), "Both arms reached target poses!");

        // /* ==============================
        //      GRIPPER CLOSE
        //    ==============================*/
        // // std::vector<double> gripper_close_l = {-0.01745, 0.15024, 1.98968, 0.86708, -0.01745, 0.31791,
        // //                                           1.84650, 0.85199, -0.01920, -0.24435, 1.87340, 0.90973};
        std::vector<double> gripper_close_l = {-0.01745, 0.0, 0.5, 0.86708,
                                            -0.01745, 0.0, 1.0, 0.85199,
                                            -0.01920, 0.0, 1.0, 0.90973};

        std::vector<std::string> joint_names = move_group_gripper_right_->getJointNames();
        std::map<std::string, double> joint_targets;
        for (size_t i = 0; i < joint_names.size(); ++i){
            joint_targets[joint_names[i]] = gripper_close_l[i];}
        move_group_gripper_right_->setJointValueTarget(joint_targets);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        auto success = (move_group_gripper_right_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success){
            move_group_gripper_right_->execute(gripper_plan);
            RCLCPP_INFO(this->get_logger(), "Gripper action successfully executed.");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Failed to plan gripper closing.");
        }

        moveit_msgs::msg::AttachedCollisionObject attach_object;
        attach_object.link_name = "R_delto_base_flange";
        attach_object.object.id = "cylinder";
        attach_object.object.operation = attach_object.object.ADD;
        planning_scene_interface.applyAttachedCollisionObject(attach_object);
        RCLCPP_WARN(this->get_logger(), "Attached Object.");

       /* ==============================
            2. MOVE TO HANDOVER
           ==============================*/
        //moveit::core::RobotStatePtr current_state = move_group_both_->getCurrentState();
        //move_group_both_->setStartState(*current_state);

        //auto left_current = move_group_left_->getCurrentJointValues();

        geometry_msgs::msg::Pose pose_left1, pose_right1;

        pose_left1.orientation.x = 0.707;
        pose_left1.orientation.y = 0.0;
        pose_left1.orientation.z = 0.0;
        pose_left1.orientation.w = 0.707;
        pose_left1.position.x = 0.45;
        pose_left1.position.y = 0.17;
        pose_left1.position.z = 0.0;

        pose_right1.orientation.x = -0.707;
        pose_right1.orientation.y = 0.0;
        pose_right1.orientation.z = 0.0;
        pose_right1.orientation.w = 0.707;
        pose_right1.position.x = 0.45;
        pose_right1.position.y = -0.17;
        pose_right1.position.z = -0.1;

        move_group_right_->setPoseTarget(pose_right1);
        moveit::planning_interface::MoveGroupInterface::Plan plan_right1;
        bool ok_r1 = (move_group_right_->plan(plan_right1) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok_r1) {
            RCLCPP_ERROR(this->get_logger(), "Right arm IK failed!");
            return;}
        auto right_joints1 = plan_right1.trajectory_.joint_trajectory.points.back().positions;

        move_group_left_->setPoseTarget(pose_left1);
        moveit::planning_interface::MoveGroupInterface::Plan plan_left1;
        bool ok_l1 = (move_group_left_->plan(plan_left1) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok_l1) {
            RCLCPP_ERROR(this->get_logger(), "Left arm IK failed!");
            return;}
        auto left_joints1 = plan_left1.trajectory_.joint_trajectory.points.back().positions;

        std::vector<double> full_joint_goal1;
        full_joint_goal1.insert(full_joint_goal1.end(),
                               left_joints1.begin(), left_joints1.end());
        full_joint_goal1.insert(full_joint_goal1.end(),
                               right_joints1.begin(), right_joints1.end());
        // moveit::core::RobotStatePtr current_state = move_group_both_->getCurrentState();
        // move_group_both_->setStartState(*current_state);
        move_group_both_->setJointValueTarget(full_joint_goal1);

        moveit::planning_interface::MoveGroupInterface::Plan plan_both1;
        bool ok_b1 = (move_group_both_->plan(plan_both1) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok_b1) {
            RCLCPP_ERROR(this->get_logger(), "Dual-arm plan failed!");
            return;}
        move_group_both_->execute(plan_both1);
        RCLCPP_INFO(this->get_logger(), "Both arms reached target poses HANDOVER!");

        /* ==============================
             GRIPPER 
           ==============================*/
        // std::vector<double> gripper_close_l = {-0.01745, 0.15024, 1.98968, 0.86708, -0.01745, 0.31791,
        //                                           1.84650, 0.85199, -0.01920, -0.24435, 1.87340, 0.90973};
      
        std::vector<std::string> joint_namesl = move_group_gripper_left_->getJointNames();
        std::map<std::string, double> joint_targets1;
        for (size_t i = 0; i < joint_namesl.size(); ++i){
            joint_targets1[joint_namesl[i]] = gripper_close_l[i];}
        move_group_gripper_left_->setJointValueTarget(joint_targets1);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_planl1;
        auto successl1 = (move_group_gripper_left_->plan(gripper_planl1) == moveit::core::MoveItErrorCode::SUCCESS);
        if (successl1){
            move_group_gripper_left_->execute(gripper_planl1);
            RCLCPP_INFO(this->get_logger(), "Gripper action successfully executed.");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Failed to plan gripper closing.");
        }
        // attach_object.link_name = "L_delto_base_flange";
        // attach_object.object.id = "cylinder";
        // attach_object.object.operation = attach_object.object.ADD;
        // planning_scene_interface.applyAttachedCollisionObject(attach_object);
        // RCLCPP_WARN(this->get_logger(), "Attached Object.");

        
        std::vector<double> gripper_open_r = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        std::map<std::string, double> joint_targetsr1;
        for (size_t i = 0; i < joint_names.size(); ++i){
            joint_targetsr1[joint_names[i]] = gripper_open_r[i];}
        move_group_gripper_right_->setJointValueTarget(joint_targetsr1);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_planr1;
        auto successr1 = (move_group_gripper_right_->plan(gripper_planr1) == moveit::core::MoveItErrorCode::SUCCESS);
        if (successr1){
            move_group_gripper_right_->execute(gripper_planr1);
            RCLCPP_INFO(this->get_logger(), "Gripper action successfully executed.");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Failed to plan gripper closing.");
        }
        attach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface.applyAttachedCollisionObject(attach_object);

        attach_object.link_name = "L_delto_base_flange";
        attach_object.object.id = "cylinder";
        attach_object.object.operation = attach_object.object.ADD;
        planning_scene_interface.applyAttachedCollisionObject(attach_object);
        RCLCPP_WARN(this->get_logger(), "Attached Object.");


        /* ==============================
            3. MOVE TO PLACE POSE
           ==============================*/
        //moveit::core::RobotStatePtr current_state = move_group_both_->getCurrentState();
        //move_group_both_->setStartState(*current_state);

        //auto left_current = move_group_left_->getCurrentJointValues();

        geometry_msgs::msg::Pose pose_left2, pose_right2;

        pose_left2.orientation.x = 0.5;
        pose_left2.orientation.y = 0.5;
        pose_left2.orientation.z = 0.5;
        pose_left2.orientation.w = 0.5;
        pose_left2.position.x = 0.51;
        pose_left2.position.y = 0.4;
        pose_left2.position.z = -0.1;

        pose_right2.orientation.x = 0.5;
        pose_right2.orientation.y = -0.5;
        pose_right2.orientation.z = 0.5;
        pose_right2.orientation.w = -0.5;
        pose_right2.position.x = 0.45;
        pose_right2.position.y = -0.17;
        pose_right2.position.z = -0.17;

        move_group_right_->setPoseTarget(pose_right2);
        moveit::planning_interface::MoveGroupInterface::Plan plan_right2;
        bool ok_r2 = (move_group_right_->plan(plan_right2) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok_r2) {
            RCLCPP_ERROR(this->get_logger(), "Right arm IK failed!");
            return;}
        auto right_joints2 = plan_right2.trajectory_.joint_trajectory.points.back().positions;

        move_group_left_->setPoseTarget(pose_left2);
        moveit::planning_interface::MoveGroupInterface::Plan plan_left2;
        bool ok_l2 = (move_group_left_->plan(plan_left2) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok_l2) {
            RCLCPP_ERROR(this->get_logger(), "Left arm IK failed!");
            return;}
        auto left_joints2 = plan_left2.trajectory_.joint_trajectory.points.back().positions;

        std::vector<double> full_joint_goal2;
        full_joint_goal2.insert(full_joint_goal2.end(),
                               left_joints2.begin(), left_joints2.end());
        full_joint_goal2.insert(full_joint_goal2.end(),
                               right_joints2.begin(), right_joints2.end());
        // moveit::core::RobotStatePtr current_state = move_group_both_->getCurrentState();
        // move_group_both_->setStartState(*current_state);
        move_group_both_->setJointValueTarget(full_joint_goal2);

        moveit::planning_interface::MoveGroupInterface::Plan plan_both2;
        bool ok_b2 = (move_group_both_->plan(plan_both2) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok_b2) {
            RCLCPP_ERROR(this->get_logger(), "Dual-arm plan failed!");
            return;}
        move_group_both_->execute(plan_both2);
        RCLCPP_INFO(this->get_logger(), "Both arms reached target poses HANDOVER!");

        /* ==============================
             GRIPPER 
           ==============================*/
        std::map<std::string, double> joint_targets2;
        for (size_t i = 0; i < joint_namesl.size(); ++i){
            joint_targets2[joint_namesl[i]] = gripper_open_r[i];}
        move_group_gripper_left_->setJointValueTarget(joint_targets2);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_planl2;
        auto successl2 = (move_group_gripper_left_->plan(gripper_planl2) == moveit::core::MoveItErrorCode::SUCCESS);
        if (successl2){
            move_group_gripper_left_->execute(gripper_planl2);
            RCLCPP_INFO(this->get_logger(), "Gripper action successfully executed.");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Failed to plan gripper closing.");
        }
        attach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface.applyAttachedCollisionObject(attach_object);

        /* ==============================
             HOME
           ==============================*/
        std::vector<std::string> joint_names_b = move_group_both_->getJointNames();
        size_t n_joints = joint_names_b.size();
        if (n_joints == 0) {
            RCLCPP_ERROR(this->get_logger(), "No joints found in arm_both!");
            return;}
        std::vector<double> zero_joints(n_joints, 0.0);
        move_group_both_->setJointValueTarget(zero_joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan_both_h;
        bool successh = (move_group_both_->plan(plan_both_h) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

        if (!successh) {
            RCLCPP_ERROR(this->get_logger(), "Planning to ZERO joints failed!");
            return;}
        move_group_both_->execute(plan_both_h);
        RCLCPP_INFO(this->get_logger(), "Both arms moved to ZERO joint positions!");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_pub_;

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
