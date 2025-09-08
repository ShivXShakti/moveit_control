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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // --------------------------
    // Define poses
    // --------------------------
    geometry_msgs::msg::Pose home_pose = move_group.getCurrentPose().pose;

    geometry_msgs::msg::Pose pregrasp_pose;
    pregrasp_pose.orientation.x = 0.4777052281279642;
    pregrasp_pose.orientation.y = 0.521342224;
    pregrasp_pose.orientation.z = 0.4777052281279642;
    pregrasp_pose.orientation.w = -0.5213422244737228;
    pregrasp_pose.position.x = 0.5;
    pregrasp_pose.position.y = 0.2815;
    pregrasp_pose.position.z = 1.3;

    geometry_msgs::msg::Pose postgrasp_pose = pregrasp_pose;
    postgrasp_pose.position.x = 0.6;  // just move in x

    // --------------------------
    // Waypoints list
    // --------------------------
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pregrasp_pose);
    waypoints.push_back(postgrasp_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;   // resolution of interpolation
    const double jump_threshold = 0.0; // disable jump threshold
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.9) {
        RCLCPP_WARN(LOGGER, "Cartesian path only planned %.2f%% of the way", fraction * 100.0);
    } else {
        RCLCPP_INFO(LOGGER, "Cartesian path planned successfully, %.2f%% achieved", fraction * 100.0);

        // Publish trajectory step by step
        rclcpp::Rate rate(10);
        RCLCPP_INFO(LOGGER, "Trajectory has %zu points", trajectory.joint_trajectory.points.size());
        for (size_t i = 0; i < trajectory.joint_trajectory.points.size() && rclcpp::ok(); ++i) {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = trajectory.joint_trajectory.points[i].positions;
            traj_pub->publish(msg);

            std::ostringstream oss;
            oss << "Published point " << i << ": ";
            for (size_t j = 0; j < msg.data.size(); ++j) {
                oss << trajectory.joint_trajectory.joint_names[j] << "=" << msg.data[j] << " ";
            }
            RCLCPP_INFO(LOGGER, "%s", oss.str().c_str());
            rate.sleep();
        }
    }

    rclcpp::shutdown();
    return 0;
}
