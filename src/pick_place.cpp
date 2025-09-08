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
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
    
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
    //move_group.execute(my_plan);
    move_group.setGoalOrientationTolerance(0.1); 

    RCLCPP_INFO(LOGGER, "Pregrasp Position");
    geometry_msgs::msg::Pose target_pose1;
    // target_pose1.orientation.x = 0.0;
    // target_pose1.orientation.y = 0.0;
    // target_pose1.orientation.z = 0.773;
    // target_pose1.orientation.w = -0.633;
    //target_pose1.orientation = rpyToQuat(0.0, 0.0, 90.0);
    target_pose1.position.x = 0.0;
    target_pose1.position.y = -1.12815;
    target_pose1.position.z = 1.6;
    RCLCPP_INFO(LOGGER, "Quat: [x=%f, y=%f, z=%f, w=%f]",
    target_pose1.orientation.x,
    target_pose1.orientation.y,
    target_pose1.orientation.z,
    target_pose1.orientation.w);
    move_group.setPoseTarget(target_pose1);
    bool Pregrasp = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (Pregrasp) {
    RCLCPP_INFO(LOGGER, "Pregrasp: Planning successful.");
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

    // RCLCPP_INFO(LOGGER, "grasp Position");
    // geometry_msgs::msg::Pose target_pose2;
    // // target_pose2.orientation.x = 0.4777052281279642;
    // // target_pose2.orientation.y = 0.521342224;
    // // target_pose2.orientation.z = 0.4777052281279642;
    // // target_pose2.orientation.w = -0.5213422244737228;
    // //target_pose2.orientation = rpyToQuat(0.0, 0.0, 140.0);
    // target_pose2.position.x = 0.6;
    // target_pose2.position.y = 0.2815;
    // target_pose2.position.z = 1.3;
    // move_group.setPlanningTime(10.0);
    // move_group.setPoseTarget(target_pose2);
    // bool grasp = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (grasp) {
    // RCLCPP_INFO(LOGGER, "grasp: Planning successful.");
    // const auto& traj = my_plan.trajectory_.joint_trajectory;
    // rclcpp::Rate rate(10); // 10 Hz
    // RCLCPP_INFO(LOGGER, "Trajectory has %zu points", traj.points.size());
    // for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
    //         std_msgs::msg::Float64MultiArray msg;
    //         msg.data = traj.points[i].positions; // copy joint positions
    //         traj_pub->publish(msg);
    //         rate.sleep();
    //     }
    // }


    // RCLCPP_INFO(LOGGER, "Postgrasp Position");
    // geometry_msgs::msg::Pose target_pose3;
    // // target_pose2.orientation.x = 0.4777052281279642;
    // // target_pose2.orientation.y = 0.521342224;
    // // target_pose2.orientation.z = 0.4777052281279642;
    // // target_pose2.orientation.w = -0.5213422244737228;
    // target_pose3.position.x = 0.6;
    // target_pose3.position.y = 0.2815;
    // target_pose3.position.z = 1.3;
    // move_group.setPlanningTime(10.0);
    // move_group.setPoseTarget(target_pose3);
    // bool Postgrasp = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (Postgrasp) {
    // RCLCPP_INFO(LOGGER, "Postgrasp: Planning successful.");
    // const auto& traj = my_plan.trajectory_.joint_trajectory;
    // rclcpp::Rate rate(10); // 10 Hz
    // RCLCPP_INFO(LOGGER, "Trajectory has %zu points", traj.points.size());
    // for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
    //         std_msgs::msg::Float64MultiArray msg;
    //         msg.data = traj.points[i].positions; // copy joint positions
    //         traj_pub->publish(msg);
    //         rate.sleep();
    //     }
    // }

    // RCLCPP_INFO(LOGGER, "Preplace Position");
    // geometry_msgs::msg::Pose target_pose4;
    // // target_pose2.orientation.x = 0.4777052281279642;
    // // target_pose2.orientation.y = 0.521342224;
    // // target_pose2.orientation.z = 0.4777052281279642;
    // // target_pose2.orientation.w = -0.5213422244737228;
    // target_pose4.position.x = 0.6;
    // target_pose4.position.y = 0.2815;
    // target_pose4.position.z = 1.3;
    // move_group.setPlanningTime(10.0);
    // move_group.setPoseTarget(target_pose4);
    // bool Preplace = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (Preplace) {
    // RCLCPP_INFO(LOGGER, "Preplace: Planning successful.");
    // const auto& traj = my_plan.trajectory_.joint_trajectory;
    // rclcpp::Rate rate(10); // 10 Hz
    // RCLCPP_INFO(LOGGER, "Trajectory has %zu points", traj.points.size());
    // for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
    //         std_msgs::msg::Float64MultiArray msg;
    //         msg.data = traj.points[i].positions; // copy joint positions
    //         traj_pub->publish(msg);
    //         rate.sleep();
    //     }
    // }

    // RCLCPP_INFO(LOGGER, "place Position");
    // geometry_msgs::msg::Pose target_pose5;
    // // target_pose2.orientation.x = 0.4777052281279642;
    // // target_pose2.orientation.y = 0.521342224;
    // // target_pose2.orientation.z = 0.4777052281279642;
    // // target_pose2.orientation.w = -0.5213422244737228;
    // target_pose5.position.x = 0.6;
    // target_pose5.position.y = 0.2815;
    // target_pose5.position.z = 1.3;
    // move_group.setPlanningTime(10.0);
    // move_group.setPoseTarget(target_pose5);
    // bool place = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (place) {
    // RCLCPP_INFO(LOGGER, "Preplace: Planning successful.");
    // const auto& traj = my_plan.trajectory_.joint_trajectory;
    // rclcpp::Rate rate(10); // 10 Hz
    // RCLCPP_INFO(LOGGER, "Trajectory has %zu points", traj.points.size());
    // for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
    //         std_msgs::msg::Float64MultiArray msg;
    //         msg.data = traj.points[i].positions; // copy joint positions
    //         traj_pub->publish(msg);
    //         rate.sleep();
    //     }
    // }

    // RCLCPP_INFO(LOGGER, "Postplace Position");
    // geometry_msgs::msg::Pose target_pose6;
    // // target_pose2.orientation.x = 0.4777052281279642;
    // // target_pose2.orientation.y = 0.521342224;
    // // target_pose2.orientation.z = 0.4777052281279642;
    // // target_pose2.orientation.w = -0.5213422244737228;
    // target_pose6.position.x = 0.6;
    // target_pose6.position.y = 0.2815;
    // target_pose6.position.z = 1.3;
    // move_group.setPlanningTime(10.0);
    // move_group.setPoseTarget(target_pose6);
    // bool Postplace = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (Postplace) {
    // RCLCPP_INFO(LOGGER, "Preplace: Planning successful.");
    // const auto& traj = my_plan.trajectory_.joint_trajectory;
    // rclcpp::Rate rate(10); // 10 Hz
    // RCLCPP_INFO(LOGGER, "Trajectory has %zu points", traj.points.size());
    // for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
    //         std_msgs::msg::Float64MultiArray msg;
    //         msg.data = traj.points[i].positions; // copy joint positions
    //         traj_pub->publish(msg);
    //         rate.sleep();
    //     }
    // }

    // std::vector<double> joint_group_positions_return;
    // current_state->copyJointGroupPositions(joint_model_group,
    // joint_group_positions_return);
    // RCLCPP_INFO(LOGGER, "Returning to home pose:");
    // joint_group_positions_return[0] = 0.0; 
    // joint_group_positions_return[1] = 0.0; 
    // joint_group_positions_return[2] = 0.0;
    // joint_group_positions_return[3] = 0.0;
    // joint_group_positions_return[4] = 0.0; 
    // joint_group_positions_return[5] = 0.0; 
    // joint_group_positions_return[6] = 0.0;
    // move_group.setJointValueTarget(joint_group_positions_return);
    // bool Returning = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (Returning) {
    // RCLCPP_INFO(LOGGER, "Return: Planning successful.");
    // const auto& traj = my_plan.trajectory_.joint_trajectory;
    // rclcpp::Rate rate(10); // 10 Hz
    // RCLCPP_INFO(LOGGER, "Trajectory has %zu points", traj.points.size());
    // for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
    //         std_msgs::msg::Float64MultiArray msg;
    //         msg.data = traj.points[i].positions; // copy joint positions
    //         traj_pub->publish(msg);
    //         rate.sleep();
    //     }
    // }

    rclcpp::shutdown();
    return 0;
}