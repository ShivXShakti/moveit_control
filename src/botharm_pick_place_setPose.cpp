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
    MoveItPlanner(): Node("pick_place"),planning_group_r("arm_right"), planning_group_l("arm_left"),pose_received_(false)
    {
        traj_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("planned_trajectory", 10);
        subscription_ = this->create_subscription<dualarm_custom_msgs::msg::ObjPoseArray>(
            "object_pose_torso", 10,
            std::bind(&MoveItPlanner::poseCallback, this, _1));
    }
    
    void setupMoveGroup()
    {
        /* ==============================
            LEFT ARM PARAMS
           ==============================*/
        move_group_l = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), planning_group_l);
        //move_group_r->setPoseReferenceFrame("torso")
        RCLCPP_INFO(this->get_logger(), "LEFTARM: Planning frame: %s", move_group_l->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "LEFTARM: End effector link: %s", move_group_l->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "LEFTARM: Pose reference frame: %s", move_group_l->getPoseReferenceFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "LEFTARM: Available Planning Groups:");
        std::copy(move_group_l->getJointModelGroupNames().begin(),
                  move_group_l->getJointModelGroupNames().end(),
                  std::ostream_iterator<std::string>(std::cout, ", "));
        
        move_group_l->setPlanningTime(10.0);
        //set planner
        //move_group_r->setPlannerId("CHOMP");
        //move_group_r->setPlannerId("RRTConnectkConfigDefault");  //ompl
        //move_group_r->setNumPlanningAttempts(5);
        //move_group_r->setPoseReferenceFrame("torso");
        move_group_g = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), "gripper_left");
        /* ==============================
            RIGHT ARM PARAMS
           ==============================*/
        move_group_r = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), planning_group_r);
        //move_group_r->setPoseReferenceFrame("torso")
        RCLCPP_INFO(this->get_logger(), "RIGHTARM: Planning frame: %s", move_group_r->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "RIGHTARM: End effector link: %s", move_group_r->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "RIGHTARM: Pose reference frame: %s", move_group_r->getPoseReferenceFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "RIGHTARM: Available Planning Groups:");
        std::copy(move_group_r->getJointModelGroupNames().begin(),
                  move_group_r->getJointModelGroupNames().end(),
                  std::ostream_iterator<std::string>(std::cout, ", "));
        
        move_group_r->setPlanningTime(10.0);
        //set planner
        //move_group_r->setPlannerId("CHOMP");
        //move_group_r->setPlannerId("RRTConnectkConfigDefault");  //ompl
        //move_group_r->setNumPlanningAttempts(5);
        //move_group_r->setPoseReferenceFrame("torso");
        
        //current ee pose
        auto lcurrent_pose_stamped = move_group_l->getCurrentPose();
        auto lpose = lcurrent_pose_stamped.pose;
        RCLCPP_INFO(this->get_logger(), "LEFTARM: Current EE Pose:");
        RCLCPP_INFO(this->get_logger(), "Position: [x: %f, y: %f, z: %f]",
                    lpose.position.x, lpose.position.y, lpose.position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: [x: %f, y: %f, z: %f, w: %f]",
                    lpose.orientation.x, lpose.orientation.y,
                    lpose.orientation.z, lpose.orientation.w);

        auto rcurrent_pose_stamped = move_group_r->getCurrentPose();
        auto rpose = rcurrent_pose_stamped.pose;
        RCLCPP_INFO(this->get_logger(), "RIGHTARM: Current EE Pose:");
        RCLCPP_INFO(this->get_logger(), "Position: [x: %f, y: %f, z: %f]",
                    rpose.position.x, rpose.position.y, rpose.position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: [x: %f, y: %f, z: %f, w: %f]",
                    rpose.orientation.x, rpose.orientation.y,
                    rpose.orientation.z, rpose.orientation.w);
    }

    void run()
    {
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(this->get_logger(), "starting trajectory planning...");
        /* ==============================
            1. MOVE TO GRASP POSE
           ==============================*/
        geometry_msgs::msg::Pose grasp_pose_l;
        grasp_pose_l.orientation.x = 0.707;
        grasp_pose_l.orientation.y = 0.0;
        grasp_pose_l.orientation.z = 0.0;
        grasp_pose_l.orientation.w = 0.707;
        grasp_pose_l.position.x = 0.45;//current_pose.pose.position.x;//0.0;
        grasp_pose_l.position.y = 0.2;//current_pose.pose.position.y;//-0.8;
        grasp_pose_l.position.z = 0.2;//current_pose.pose.position.z;//-0.1;
        move_group_l->setPoseTarget(grasp_pose_l);
        moveit::planning_interface::MoveGroupInterface::Plan plan_grasp_l;
        auto success_grasp_l = (move_group_l->plan(plan_grasp_l) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success_grasp_l) {move_group_l->execute(plan_grasp_l);
            RCLCPP_INFO(this->get_logger(), "========LEFTARM: Reached grasp pose==========.");}
        else{
            RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");}

        
        geometry_msgs::msg::Pose grasp_pose_r;
        // grasp_pose_r.orientation.x = 0.5;
        // grasp_pose_r.orientation.y = -0.5;
        // grasp_pose_r.orientation.z = 0.5;
        // grasp_pose_r.orientation.w = -0.5;
        grasp_pose_r.orientation.x = -0.707;
        grasp_pose_r.orientation.y = 0.0;
        grasp_pose_r.orientation.z = 0.0;
        grasp_pose_r.orientation.w = 0.707;
        grasp_pose_r.position.x = 0.45;//current_pose.pose.position.x;//0.0;
        grasp_pose_r.position.y = -0.2;//current_pose.pose.position.y;//-0.8;
        grasp_pose_r.position.z = -0.1;//current_pose.pose.position.z;//-0.1;
        move_group_r->setPoseTarget(grasp_pose_r);
        moveit::planning_interface::MoveGroupInterface::Plan plan_grasp_r;
        auto success_grasp_r = (move_group_r->plan(plan_grasp_r) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success_grasp_r) {move_group_r->execute(plan_grasp_r);
            RCLCPP_INFO(this->get_logger(), "========RIGHTARM: Reached grasp pose==========.");}
        else{
            RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");}
        /* ==============================
            2. GRIPPER CLOSE
           ==============================*/
        // std::vector<double> gripper_close_l = {-0.01745, 0.15024, 1.98968, 0.86708, -0.01745, 0.31791,
        //                                           1.84650, 0.85199, -0.01920, -0.24435, 1.87340, 0.90973};
        std::vector<double> gripper_close_l = {-0.01745, 0.0, 1.0, 0.86708,
                                            -0.01745, 0.0, 1.0, 0.85199,
                                            -0.01920, 0.0, 1.0, 0.90973};
        //std::string gripperl = "gripper_left";
        //Gripper(gripper_close_l, gripperl);
        //RCLCPP_INFO(this->get_logger(), "=======LEFT Gripper closed======.");

        std::vector<std::string> joint_names = move_group_g->getJointNames();
        std::map<std::string, double> joint_targets;
        for (size_t i = 0; i < joint_names.size(); ++i){
            joint_targets[joint_names[i]] = gripper_close_l[i];}
        move_group_g->setJointValueTarget(joint_targets);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        auto success = (move_group_g->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success){
            move_group_g->execute(gripper_plan);
            RCLCPP_INFO(this->get_logger(), "Gripper action successfully executed.");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Failed to plan gripper closing.");
        }

        // std::vector<double> gripper_close_r = {-0.01745, 0.0, 1.0, 0.86708,
        //                                     -0.01745, 0.0, 1.0, 0.85199,
        //                                     -0.01920, 0.0, 1.0, 0.90973};
        // std::string gripperr = "gripper_right";
        // Gripper(gripper_close_r, gripperr);
        // RCLCPP_INFO(this->get_logger(), "=======RIGHT Gripper closed======.");
        
        /* ==============================
            3. ATTACH OBJECT TO GRIPPER
           ==============================*/
        // moveit_msgs::msg::AttachedCollisionObject attach_object;
        // attach_object.link_name = "R_delto_base_flange";
        // attach_object.object.id = "bottle";
        // attach_object.object.operation = attach_object.object.ADD;
        // planning_scene_interface.applyAttachedCollisionObject(attach_object);

        /* ==============================
            4. MOVE TO POST GRASP POSE
           ==============================*/
        // geometry_msgs::msg::Pose post_pose_r;
        // post_pose_r.orientation.x = -0.707;
        // post_pose_r.orientation.y = 0.0;
        // post_pose_r.orientation.z = 0.0;
        // post_pose_r.orientation.w = 0.707;
        // post_pose_r.position.x = 0.45;//current_pose.pose.position.x;//0.0;
        // post_pose_r.position.y = -0.25;//current_pose.pose.position.y;//-0.8;
        // post_pose_r.position.z = 0.2;//current_pose.pose.position.z;//-0.1;
        // move_group_r->setPoseTarget(post_pose_r);
        // moveit::planning_interface::MoveGroupInterface::Plan plan_post_r;
        // auto success_post_r = (move_group_r->plan(plan_post_r) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // if (success_post_r) {move_group_r->execute(plan_post_r);
        //     RCLCPP_INFO(this->get_logger(), "========RIGHTARM: Reached post pose==========.");}
        // else{
        //     RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");}

        // /* ==============================
        //     5. MOVE TO PLACE POSE
        //    ==============================*/
        // geometry_msgs::msg::Pose place_pose;
        // place_pose.orientation.x = 0.5;
        // place_pose.orientation.y = -0.5;
        // place_pose.orientation.z = 0.5;
        // place_pose.orientation.w = -0.5;
        // place_pose.position.x = 0.51;
        // place_pose.position.y = -0.4;
        // place_pose.position.z = -0.28;
        // move_group_r->setPoseTarget(place_pose);
        // moveit::planning_interface::MoveGroupInterface::Plan plan_place;
        // auto success_place = (move_group_r->plan(plan_place) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // if (success_place) {move_group_r->execute(plan_place);
        //     RCLCPP_INFO(this->get_logger(), "========Reached place pose==========.");}
        // else{RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");}
        
        // /* ==============================
        //     6. GRIPPER OPEN
        //    ==============================*/
        // std::vector<double> gripper_open = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        // Gripper(gripper_open);
        // RCLCPP_INFO(this->get_logger(), "========Gripper Open==========.");

        // /* ==============================
        //     7. DETACH OBJECT
        //    ==============================*/
        // attach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        // planning_scene_interface.applyAttachedCollisionObject(attach_object);
        // //planning_scene_interface.removeCollisionObjects({"cube"});
        
        // /* ==============================
        //     8. MOVE TO HOME POSE
        //    ==============================*/
        // geometry_msgs::msg::Pose home_pose;
        // home_pose.orientation.x = 0.5;
        // home_pose.orientation.y = -0.5;
        // home_pose.orientation.z = 0.5;
        // home_pose.orientation.w = -0.5;
        // home_pose.position.x = 0.55;
        // home_pose.position.y = -0.5;
        // home_pose.position.z = 0.0;
        // move_group_r->setPoseTarget(home_pose);
        // moveit::planning_interface::MoveGroupInterface::Plan plan_home;
        // auto success_home = (move_group_r->plan(plan_home) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // if (success_home) {move_group_r->execute(plan_home);
        //     RCLCPP_INFO(this->get_logger(), "========Reached home pose==========.");}
        // else{RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");}
    }

private:
    double deg2rad(double deg) { return deg * M_PI / 180.0; }
    
    void Gripper(const std::vector<double> &gripper_joint_values, std::string &gripper){
        if (gripper_joint_values.size() != 12){
            RCLCPP_ERROR(this->get_logger(), "Gripper joint values vector must have 12 elements.");
            return;
        }
        auto node = rclcpp::Node::make_shared("tmp_gripper_node");
        moveit::planning_interface::MoveGroupInterface gripper_group(node, gripper);
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

    std::string planning_group_r;
    std::string planning_group_l;
    //std::string move_group_g;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_l;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_r;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_g;

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
