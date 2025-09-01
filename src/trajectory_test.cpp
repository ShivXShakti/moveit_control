#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "left_arm";

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

    joint_group_positions[0] = 0.01; 
    joint_group_positions[1] = -0.01; 
    joint_group_positions[2] = 0.01;
    joint_group_positions[3] = 0.01;
    joint_group_positions[4] = -0.01; 
    joint_group_positions[5] = 0.01; 
    joint_group_positions[6] = 0.001;
    move_group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
    RCLCPP_INFO(LOGGER, "Planning successful. Trajectory:");

    const auto& traj = my_plan.trajectory_.joint_trajectory;
    rclcpp::Rate rate(10); // 10 Hz

    RCLCPP_INFO(LOGGER, "Trajectory has %zu points", traj.points.size());

    for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = traj.points[i].positions; // copy joint positions

            traj_pub->publish(msg);

            std::ostringstream oss;
            oss << "Published point " << i << ": ";
            for (size_t j = 0; j < msg.data.size(); ++j) {
                oss << traj.joint_names[j] << "=" << msg.data[j] << " ";
            }
            RCLCPP_INFO(LOGGER, "%s", oss.str().c_str());

            rate.sleep();
        }
    }
    //move_group.execute(my_plan);

    RCLCPP_INFO(LOGGER, "Pregrasp Position");
    geometry_msgs::msg::Pose target_pose1;
    //target_pose1.orientation.x = -1.0;
    //target_pose1.orientation.y = 0.00;
    //target_pose1.orientation.z = 0.00;
    //target_pose1.orientation.w = 0.00;
    target_pose1.position.x = 0.3;
    target_pose1.position.y = 0.3;
    target_pose1.position.z = 1.3;
    move_group.setPoseTarget(target_pose1);
    bool success_arm = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success_arm) {
    RCLCPP_INFO(LOGGER, "Planning successful. Trajectory:");

    const auto& traj = my_plan.trajectory_.joint_trajectory;
    rclcpp::Rate rate(10); // 10 Hz

    RCLCPP_INFO(LOGGER, "Trajectory has %zu points", traj.points.size());

    for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = traj.points[i].positions; // copy joint positions

            traj_pub->publish(msg);

            std::ostringstream oss;
            oss << "Published point " << i << ": ";
            for (size_t j = 0; j < msg.data.size(); ++j) {
                oss << traj.joint_names[j] << "=" << msg.data[j] << " ";
            }
            RCLCPP_INFO(LOGGER, "%s", oss.str().c_str());

            rate.sleep();
        }
    }
    

   
    RCLCPP_INFO(LOGGER, "Current joint positions:");
    for (std::size_t i = 0; i < joint_group_positions.size(); ++i) {
    RCLCPP_INFO(LOGGER, "  Joint %zu: %f", i, joint_group_positions[i]);
    }
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(LOGGER, "Current pose: position(%.3f, %.3f, %.3f), orientation(%.3f, %.3f, %.3f, %.3f)",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z,
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w);
    rclcpp::shutdown();
    return 0;
}