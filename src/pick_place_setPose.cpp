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


using std::placeholders::_1;
using namespace std::chrono_literals;

class MoveItPlanner : public rclcpp::Node{
public:
    MoveItPlanner(): Node("pick_place"),planning_group_("arm_right"),pose_received_(false)
    {
        traj_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("planned_trajectory", 10);
        subscription_ = this->create_subscription<dualarm_custom_msgs::msg::ObjPoseArray>(
            "object_pose_torso", 10,
            std::bind(&MoveItPlanner::poseCallback, this, _1));
    }
    
    void setupMoveGroup()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), planning_group_);
        //move_group_->setPoseReferenceFrame("torso")
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
        std::copy(move_group_->getJointModelGroupNames().begin(),
                  move_group_->getJointModelGroupNames().end(),
                  std::ostream_iterator<std::string>(std::cout, ", "));
        
        move_group_->setPlanningTime(10.0);
        //set planner
        //move_group_->setPlannerId("CHOMP");
        //move_group_->setPlannerId("RRTConnectkConfigDefault");  //ompl
        //move_group_->setNumPlanningAttempts(5);
        //move_group_->setPoseReferenceFrame("torso");
        
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
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(this->get_logger(), "starting trajectory planning...");
        /* ==============================
            1. MOVE TO GRASP POSE
           ==============================*/
        geometry_msgs::msg::Pose grasp_pose;
        grasp_pose.orientation.x = 0.5;
        grasp_pose.orientation.y = -0.5;
        grasp_pose.orientation.z = 0.5;
        grasp_pose.orientation.w = -0.5;
        grasp_pose.position.x = 0.51;//current_pose.pose.position.x;//0.0;
        grasp_pose.position.y = -0.4;//current_pose.pose.position.y;//-0.8;
        grasp_pose.position.z = -0.3;//current_pose.pose.position.z;//-0.1;
        move_group_->setPoseTarget(grasp_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_grasp;
        auto success_grasp = (move_group_->plan(plan_grasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success_grasp) {move_group_->execute(plan_grasp);
            RCLCPP_INFO(this->get_logger(), "========Reached grasp pose==========.");}
        else{
            RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");}
       
        /* ==============================
            2. GRIPPER CLOSE
           ==============================*/
        // std::vector<double> gripper_close = {-0.01745, 0.15024, 1.98968, 0.86708, -0.01745, 0.31791,
        //                                           1.84650, 0.85199, -0.01920, -0.24435, 1.87340, 0.90973};
        std::vector<double> gripper_close = {-0.01745, 0.15024, 1.0, 0.86708,
                                            -0.01745, 0.31791, 1.0, 0.85199,
                                            -0.01920, -0.24435, 1.0, 0.90973};
        Gripper(gripper_close);
        RCLCPP_INFO(this->get_logger(), "=======Gripper closed======.");
        
        /* ==============================
            3. ATTACH OBJECT TO GRIPPER
           ==============================*/
        moveit_msgs::msg::AttachedCollisionObject attach_object;
        attach_object.link_name = "R_delto_base_flange";
        attach_object.object.id = "bottle";
        attach_object.object.operation = attach_object.object.ADD;
        planning_scene_interface.applyAttachedCollisionObject(attach_object);

        // /* ==============================
        //     4. MOVE TO POST GRASP POSE
        //    ==============================*/
        // geometry_msgs::msg::Pose post_grasp_pose;
        // post_grasp_pose.orientation.x = 0.5;
        // post_grasp_pose.orientation.y = -0.5;
        // post_grasp_pose.orientation.z = 0.5;
        // post_grasp_pose.orientation.w = -0.5;
        // post_grasp_pose.position.x = 0.51;
        // post_grasp_pose.position.y = -0.4;
        // post_grasp_pose.position.z = -0.1;
        // move_group_->setPoseTarget(post_grasp_pose);
        // moveit::planning_interface::MoveGroupInterface::Plan plan_post_grasp;
        // auto success_post_grasp = (move_group_->plan(plan_post_grasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // if (success_post_grasp) {move_group_->execute(plan_post_grasp);
        //     RCLCPP_INFO(this->get_logger(), "========Reached post grasp pose==========.");}
        // else{RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");}

        /* ==============================
            5. MOVE TO PLACE POSE
           ==============================*/
        geometry_msgs::msg::Pose place_pose;
        place_pose.orientation.x = 0.5;
        place_pose.orientation.y = -0.5;
        place_pose.orientation.z = 0.5;
        place_pose.orientation.w = -0.5;
        place_pose.position.x = 0.51;
        place_pose.position.y = -0.4;
        place_pose.position.z = -0.28;
        move_group_->setPoseTarget(place_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_place;
        auto success_place = (move_group_->plan(plan_place) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success_place) {move_group_->execute(plan_place);
            RCLCPP_INFO(this->get_logger(), "========Reached place pose==========.");}
        else{RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");}
        
        /* ==============================
            6. GRIPPER OPEN
           ==============================*/
        std::vector<double> gripper_open = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        Gripper(gripper_open);
        RCLCPP_INFO(this->get_logger(), "========Gripper Open==========.");

        /* ==============================
            7. DETACH OBJECT
           ==============================*/
        attach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface.applyAttachedCollisionObject(attach_object);
        //planning_scene_interface.removeCollisionObjects({"cube"});
        
        /* ==============================
            8. MOVE TO HOME POSE
           ==============================*/
        geometry_msgs::msg::Pose home_pose;
        home_pose.orientation.x = 0.5;
        home_pose.orientation.y = -0.5;
        home_pose.orientation.z = 0.5;
        home_pose.orientation.w = -0.5;
        home_pose.position.x = 0.55;
        home_pose.position.y = -0.5;
        home_pose.position.z = 0.0;
        move_group_->setPoseTarget(home_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_home;
        auto success_home = (move_group_->plan(plan_home) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success_home) {move_group_->execute(plan_home);
            RCLCPP_INFO(this->get_logger(), "========Reached home pose==========.");}
        else{RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");}
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

    void publishTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory){
        rclcpp::Rate rate(10);
        for (size_t i = 0; i < trajectory.joint_trajectory.points.size() && rclcpp::ok(); ++i){
            std_msgs::msg::Float64MultiArray msg;
            msg.data = trajectory.joint_trajectory.points[i].positions;
            traj_pub_->publish(msg);
            rate.sleep();
        }
    }

    std::string planning_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_pub_;
    rclcpp::Subscription<dualarm_custom_msgs::msg::ObjPoseArray>::SharedPtr subscription_;

    geometry_msgs::msg::Pose first_pose_;
    bool pose_received_;
    std::vector<double> my_gripper_values;
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
