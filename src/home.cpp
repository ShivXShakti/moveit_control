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
static const std::string PLANNING_GROUP_L = "left_arm";
static const std::string PLANNING_GROUP_R = "right_arm";

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

    auto traj_publ = move_group_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "left/planned_trajectory", 10);
auto traj_pubr = move_group_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "right/planned_trajectory", 10);
    auto traj_pub = move_group_node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "combined/planned_trajectory", 10);
    
    moveit::planning_interface::MoveGroupInterface move_group_l(move_group_node, PLANNING_GROUP_L);
    moveit::planning_interface::MoveGroupInterface move_group_r(move_group_node, PLANNING_GROUP_R);

    
    const moveit::core::JointModelGroup *joint_model_groupl = move_group_l.getCurrentState()->getJointModelGroup(PLANNING_GROUP_L);
     const moveit::core::JointModelGroup *joint_model_groupr = move_group_r.getCurrentState()->getJointModelGroup(PLANNING_GROUP_R);
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_l.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_l.getEndEffectorLink().c_str());
    
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_l.getJointModelGroupNames().begin(),move_group_l.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));
    std::copy(move_group_r.getJointModelGroupNames().begin(),move_group_r.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));
    
    moveit::core::RobotStatePtr current_statel = move_group_l.getCurrentState(100);
    moveit::core::RobotStatePtr current_stater = move_group_r.getCurrentState(100);
    
    std::vector<double> joint_group_positionsl, joint_group_positionsr;
    current_statel->copyJointGroupPositions(joint_model_groupl,
    joint_group_positionsl);
    current_stater->copyJointGroupPositions(joint_model_groupr,
    joint_group_positionsr);
    
    RCLCPP_INFO(LOGGER, "Current joint positions:");
    // for (std::size_t i = 0; i < joint_group_positions.size(); ++i) {
    // RCLCPP_INFO(LOGGER, "  Joint %zu: %f", i, joint_group_positions[i]);
    // }

    joint_group_positionsl[0] = 0.0; 
    joint_group_positionsl[1] = 0.0; 
    joint_group_positionsl[2] = 0.0;
    joint_group_positionsl[3] = 0.0;
    joint_group_positionsl[4] = 0.0; 
    joint_group_positionsl[5] = 0.0; 
    joint_group_positionsl[6] = 0.0;
    joint_group_positionsr[0] = 0.0; 
    joint_group_positionsr[1] = 0.0; 
    joint_group_positionsr[2] = 0.0;
    joint_group_positionsr[3] = 0.0;
    joint_group_positionsr[4] = 0.0; 
    joint_group_positionsr[5] = 0.0; 
    joint_group_positionsr[6] = 0.0;

    move_group_l.setJointValueTarget(joint_group_positionsl);
    moveit::planning_interface::MoveGroupInterface::Plan plan_l;
    bool successl = (move_group_l.plan(plan_l) == moveit::core::MoveItErrorCode::SUCCESS);

    move_group_r.setJointValueTarget(joint_group_positionsr);
    moveit::planning_interface::MoveGroupInterface::Plan plan_r;
    bool successr = (move_group_r.plan(plan_r) == moveit::core::MoveItErrorCode::SUCCESS);

    if (successl && successr) {
    RCLCPP_INFO(LOGGER, "Home: Planning successful.");

    const auto& trajl = plan_l.trajectory_.joint_trajectory;
    const auto& trajr = plan_r.trajectory_.joint_trajectory;
    // traj_publ->publish(plan_l.trajectory_.joint_trajectory);
    // traj_pubr->publish(plan_r.trajectory_.joint_trajectory);
    size_t max_points = std::max(trajl.points.size(), trajr.points.size());
    rclcpp::Rate rate(10);  // 10 Hz

    for (size_t i = 0; i < max_points && rclcpp::ok(); ++i) {
        std_msgs::msg::Float64MultiArray msg;

        // Left arm: if finished, repeat the last point
        std::vector<double> left_positions;
        if (i < trajl.points.size()) {
            left_positions = trajl.points[i].positions;
        } else {
            left_positions = trajl.points.back().positions;
        }

        // Right arm: if finished, repeat the last point
        std::vector<double> right_positions;
        if (i < trajr.points.size()) {
            right_positions = trajr.points[i].positions;
        } else {
            right_positions = trajr.points.back().positions;
        }

        // Concatenate left + right
        msg.data.insert(msg.data.end(), left_positions.begin(), left_positions.end());
        msg.data.insert(msg.data.end(), right_positions.begin(), right_positions.end());

        // Publish one combined message
        traj_pub->publish(msg);

        rate.sleep();
    }

        }
    rclcpp::shutdown();
    return 0;
}