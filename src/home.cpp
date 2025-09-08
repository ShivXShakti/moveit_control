#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <sstream>
#include <cmath> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "right_arm";

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

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


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("home", node_options);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto traj_pub = move_group_node->create_publisher<std_msgs::msg::Float64MultiArray>("planned_trajectory", 10);
    
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    
    const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));
    
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(100);
    
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,
    joint_group_positions);
    
    RCLCPP_INFO(LOGGER, "Current joint positions:");
    for (std::size_t i = 0; i < joint_group_positions.size(); ++i) {
    RCLCPP_INFO(LOGGER, "  Joint %zu: %f", i, joint_group_positions[i]);
    }

    joint_group_positions[0] = 0.0; 
    joint_group_positions[1] = 0.0; 
    joint_group_positions[2] = 0.0;
    joint_group_positions[3] = 0.0;
    joint_group_positions[4] = 0.0; 
    joint_group_positions[5] = 0.0; 
    joint_group_positions[6] = 0.0;
    move_group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
    RCLCPP_INFO(LOGGER, "Home: Planning successful.");

    const auto& traj = my_plan.trajectory_.joint_trajectory;
    rclcpp::Rate rate(10); // 10 Hz
    RCLCPP_INFO(LOGGER, "Trajectory has %zu points", traj.points.size());
    for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = traj.points[i].positions; // copy joint positions
            traj_pub->publish(msg);
            rate.sleep();
        }
    }
    rclcpp::shutdown();
    return 0;
}