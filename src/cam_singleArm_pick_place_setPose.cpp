#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <sstream>
#include <cmath>
#include <iterator>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <dualarm_custom_msgs/msg/obj_pose_array.hpp>
#include <dualarm_custom_msgs/msg/obj_pose.hpp>

#include <darm_msgs/msg/ui_command.hpp>
#include <darm_msgs/msg/ui_status.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class MoveItPlanner : public rclcpp::Node{
public:
    MoveItPlanner(): Node("pick_place"),planning_group_("arm_left"),pose_received_(false)
    {
        subscription_ = this->create_subscription<dualarm_custom_msgs::msg::ObjPoseArray>(
            "object_pose_torso", 10,
            std::bind(&MoveItPlanner::poseCallback, this, _1));
        pub_hw = this->create_publisher<darm_msgs::msg::UiCommand>("/svaya/ui/command", 10);

        pub_hw_freq = 20.0;
    }
    
    void setupMoveGroup()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), planning_group_);
        move_group_->setPoseReferenceFrame("torso_hw");
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
        std::copy(move_group_->getJointModelGroupNames().begin(),
                  move_group_->getJointModelGroupNames().end(),
                  std::ostream_iterator<std::string>(std::cout, ", "));
        
        move_group_->setPlanningTime(10.0);
        //set planner
        move_group_->setPlannerId("CHOMP");
        //move_group_->setPlannerId("RRTConnectkConfigDefault");  //ompl
        //move_group_->setNumPlanningAttempts(5);
        
        move_group_gripper_left_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), "gripper_left");

        //current ee pose
        auto current_pose_stamped = move_group_->getCurrentPose();
        auto pose = current_pose_stamped.pose;

        RCLCPP_INFO(this->get_logger(), "Current EE Pose:");
        RCLCPP_INFO(this->get_logger(), "Position: [x: %f, y: %f, z: %f]",
                    pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: [x: %f, y: %f, z: %f, w: %f]",
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w);
    }

    void run()
    {
        rclcpp::Rate wait_rate(10);
        while (rclcpp::ok() && !pose_received_) {
            wait_rate.sleep();
        }
        if (!rclcpp::ok()) return;

        RCLCPP_INFO(this->get_logger(), "First pose received, starting trajectory planning...");
        /* ==============================
            1. MOVE TO GRASP POSE
           ==============================*/
        geometry_msgs::msg::Pose grasp_pose;
        grasp_pose.orientation.x = 0.5;
        grasp_pose.orientation.y = 0.5;
        grasp_pose.orientation.z = 0.5;
        grasp_pose.orientation.w = 0.5;
        grasp_pose.position = first_pose_.position;
        move_group_->setPoseTarget(grasp_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_grasp;
        auto success_grasp = (move_group_->plan(plan_grasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success_grasp) {
            move_group_->execute(plan_grasp);
            //execute_hw(plan_grasp);
            RCLCPP_INFO(this->get_logger(), "========Reached grasp pose==========.");}
        else{
            RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");
            return;
        }
         /* ==============================
             GRIPPER 
           ==============================*/
        // // std::vector<double> gripper_close_l = {-0.01745, 0.15024, 1.98968, 0.86708, -0.01745, 0.31791,
        // //                                           1.84650, 0.85199, -0.01920, -0.24435, 1.87340, 0.90973};
        std::vector<double> gripper_close_l = {-0.01745, 0.0, 0.5, 0.86708,
                                            -0.01745, 0.0, 1.0, 0.85199,
                                            -0.01920, 0.0, 1.0, 0.90973};

        std::vector<std::string> joint_names = move_group_gripper_left_->getJointNames();
        std::map<std::string, double> joint_targets;
        for (size_t i = 0; i < joint_names.size(); ++i){
            joint_targets[joint_names[i]] = gripper_close_l[i];}
        move_group_gripper_left_->setJointValueTarget(joint_targets);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        auto successC1 = (move_group_gripper_left_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (successC1){
            move_group_gripper_left_->execute(gripper_plan);
            RCLCPP_INFO(this->get_logger(), "Gripper action successfully executed.");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Failed to plan gripper closing.");
            return;
        }

        moveit_msgs::msg::AttachedCollisionObject attach_object;
        attach_object.link_name = "L_delto_base_flange";
        attach_object.object.id = "bottle";
        attach_object.object.operation = attach_object.object.ADD;
        planning_scene_interface.applyAttachedCollisionObject(attach_object);
        RCLCPP_WARN(this->get_logger(), "Attached Object.");

        /* ==============================
            2. MOVE TO PLACE POSE
           ==============================*/
        geometry_msgs::msg::Pose place_pose;
        place_pose.orientation.x = 0.5;
        place_pose.orientation.y = 0.5;
        place_pose.orientation.z = 0.5;
        place_pose.orientation.w = 0.5;
        place_pose.position.x = 0.5;
        place_pose.position.y = 0.3;
        place_pose.position.z = -0.45;
        move_group_->setPoseTarget(place_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_place;
        auto success_place = (move_group_->plan(plan_place) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success_place) {
            move_group_->execute(plan_place);
            //execute_hw(plan_place);
            RCLCPP_INFO(this->get_logger(), "========Reached grasp pose==========.");}
        else{
            RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");
            return;}

        /* ==============================
            GRIPPER 
        ==============================*/
        std::vector<double> gripper_open_r = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        std::map<std::string, double> joint_targetsr1;
        for (size_t i = 0; i < joint_names.size(); ++i){
            joint_targetsr1[joint_names[i]] = gripper_open_r[i];}
        move_group_gripper_left_->setJointValueTarget(joint_targetsr1);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_planr1;
        auto successr1 = (move_group_gripper_left_->plan(gripper_planr1) == moveit::core::MoveItErrorCode::SUCCESS);
        if (successr1){
            move_group_gripper_left_->execute(gripper_planr1);
            RCLCPP_INFO(this->get_logger(), "Gripper action successfully executed.");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Failed to plan gripper closing.");
            return;
        }
        attach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface.applyAttachedCollisionObject(attach_object);
        
        /* ==============================
            3. MOVE TO home
           ==============================*/
        std::vector<std::string> arm_joint_names = move_group_->getJointNames();
        size_t n_joints = arm_joint_names.size();

        if (n_joints == 0) {
            RCLCPP_ERROR(this->get_logger(), "No joints found in arm_both!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "arm_both has %zu joints", n_joints);
        std::vector<double> zero_joints(n_joints, 0.0);
        move_group_->setJointValueTarget(zero_joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan_both;
        bool success = (move_group_->plan(plan_both) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group_->execute(plan_both);
        }else{
            RCLCPP_ERROR(this->get_logger(), "Planning to ZERO joints failed!");
            return;
        }
        
        //execute_hw(plan_both);    
    }

private:
    double deg2rad(double deg) { return deg * M_PI / 180.0; }
    
    void Gripper(const std::vector<double> &gripper_joint_values){
        if (gripper_joint_values.size() != 12){
            RCLCPP_ERROR(this->get_logger(), "Gripper joint values vector must have 12 elements.");
            return;
        }
        auto node = rclcpp::Node::make_shared("tmp_gripper_node");
        moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper_right");
        std::vector<std::string> joint_names = gripper_group.getJointNames();
        if (joint_names.size() != 12){
            RCLCPP_ERROR(this->get_logger(), "Gripper MoveGroup has %zu joints, expected 12.", joint_names.size());
            return;
        }
        std::map<std::string, double> joint_targets;
        for (size_t i = 0; i < joint_names.size(); ++i){
            joint_targets[joint_names[i]] = gripper_joint_values[i];
        }
        gripper_group.setJointValueTarget(joint_targets);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        auto success = (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success){
            gripper_group.execute(gripper_plan);
            RCLCPP_INFO(this->get_logger(), "Gripper action successfully executed.");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Failed to plan gripper closing.");
        }
    }

    void poseCallback(const dualarm_custom_msgs::msg::ObjPoseArray::SharedPtr msg){
        if (pose_received_) return;
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty ObjPoseArray.");
            return;
        }
        const auto &obj = msg->data.front();
        const auto &pose = obj.pose_stamped.pose;

        first_pose_ = pose; 
        pose_received_ = true;
        RCLCPP_INFO(this->get_logger(),
            "🟢 Stored first object: %s | Position [x: %.3f, y: %.3f, z: %.3f]",
            obj.object_name.c_str(), pose.position.x, pose.position.y, pose.position.z);
    }

    void execute_hw(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        rclcpp::Rate rate(pub_hw_freq);  // 10 Hz streaming

        auto msg = darm_msgs::msg::UiCommand();
        msg.developer_command.enable = true;

        const auto &traj = plan.trajectory_.joint_trajectory;
        const size_t num_joints = traj.joint_names.size();

        msg.developer_command.command.resize(2*num_joints+2);

        for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i)
        {
            const auto &pt = traj.points[i];

            for (size_t j = 0; j < num_joints; ++j)
            {
                msg.developer_command.command[j].position = pt.positions[j];
                if (!pt.velocities.empty())
                    msg.developer_command.command[j].velocity = pt.velocities[j];
                else
                    msg.developer_command.command[j].velocity = 0.0;
                
                msg.developer_command.command[j+7].position = 0.0;
                msg.developer_command.command[j+7].velocity = 0.0;
                
                if (j < 2){
                    msg.developer_command.command[j+14].position = 0.0;
                    msg.developer_command.command[j+14].velocity = 0.0;}
                }
            pub_hw->publish(msg);
            std::cout << "published joint " << i+1 << ": " << msg.developer_command.command[0].position << ", "<< msg.developer_command.command[1].position << ", "<< msg.developer_command.command[2].position << ", "<< msg.developer_command.command[3].position << ", "<< msg.developer_command.command[4].position << ", "<< msg.developer_command.command[5].position << ", "<< msg.developer_command.command[6].position << ", "
            << msg.developer_command.command[7].position << ", "<< msg.developer_command.command[8].position << ", "<< msg.developer_command.command[9].position << ", "<< msg.developer_command.command[10].position << ", "<< msg.developer_command.command[11].position << ", "<< msg.developer_command.command[12].position << ", "<< msg.developer_command.command[13].position << ", "<< msg.developer_command.command[14].position << ", "<<msg.developer_command.command[15].position << " rad" << std::endl;
            rate.sleep();
        }
    }

    std::string planning_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_left_;

    rclcpp::Subscription<dualarm_custom_msgs::msg::ObjPoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<darm_msgs::msg::UiCommand>::SharedPtr pub_hw;

    geometry_msgs::msg::Pose first_pose_;
    bool pose_received_;
    float pub_hw_freq;
};


// ---------------- DRIVER FUNCTION ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MoveItPlanner>();
    node->setupMoveGroup();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    node->run();

    rclcpp::shutdown();
    return 0;
}
