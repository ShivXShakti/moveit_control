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
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <dualarm_custom_msgs/msg/obj_pose_array.hpp>
#include <dualarm_custom_msgs/msg/obj_pose.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MoveItPlanner : public rclcpp::Node
{
public:
    MoveItPlanner(): Node("pick_place"),planning_group_("arm_left"),pose_received_(false)
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
        RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
        std::copy(move_group_->getJointModelGroupNames().begin(),
                  move_group_->getJointModelGroupNames().end(),
                  std::ostream_iterator<std::string>(std::cout, ", "));
        
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


        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;
        const double jump_threshold = 0.0;

        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = 0.000032; //rpyToQuat(135.0, 0.0, 180.0);
        target_pose.orientation.y = 0.707105;
        target_pose.orientation.z = 0.707105;
        target_pose.orientation.w = 0.000104;
        target_pose.position.x = 0.0;//first_pose_.position;
        target_pose.position.y = 1.13455;
        target_pose.position.z = 0.0;

        std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};

        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        if (fraction>=0.9){
            auto result = move_group_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Cartesian path planned: , %.2f%% achieved", fraction * 100.0);
            if (result == moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_INFO(this->get_logger(), "EXECUTION COMPLETED");}
            else {
                RCLCPP_INFO(this->get_logger(), "Error");
            }
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Cartesian path planned: , %.2f%% achieved", fraction * 100.0);
        }
        publishTrajectory(trajectory);
    }

private:
    double deg2rad(double deg) { return deg * M_PI / 180.0; }

    geometry_msgs::msg::Quaternion rpyToQuat(double roll, double pitch, double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
        q.normalize();

        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();
        return q_msg;
    }

    void poseCallback(const dualarm_custom_msgs::msg::ObjPoseArray::SharedPtr msg)
    {
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

    void publishTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory)
    {
        rclcpp::Rate rate(10);
        for (size_t i = 0; i < trajectory.joint_trajectory.points.size() && rclcpp::ok(); ++i)
        {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = trajectory.joint_trajectory.points[i].positions;
            traj_pub_->publish(msg);
            rate.sleep();
        }
    }

    std::string planning_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_pub_;
    rclcpp::Subscription<dualarm_custom_msgs::msg::ObjPoseArray>::SharedPtr subscription_;

    geometry_msgs::msg::Pose first_pose_;
    bool pose_received_;
};


// ---------------- MAIN FUNCTION ----------------
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
