#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <sstream>
#include <cmath>  // for M_PI

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
    auto move_group_node = rclcpp::Node::make_shared("pick_place", node_options);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto traj_pub = move_group_node->create_publisher<std_msgs::msg::Float64MultiArray>("planned_trajectory", 10);
    
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    
    //const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));
    
    /*-------------------------------------------------------------------
                                DEFINE POSES
    -----------------------------------------------------------------------*/

    geometry_msgs::msg::Pose pregrasp_posel, pregrasp_poser;
    pregrasp_posel.orientation = rpyToQuat(50.0, 0.0, 0.0);
    pregrasp_posel.position.x = 0.5;
    pregrasp_posel.position.y = 0.3;
    pregrasp_posel.position.z = 1.3;
    pregrasp_poser.orientation = rpyToQuat(50.0, 0.0, 0.0);
    pregrasp_poser.position.x = 0.5;
    pregrasp_poser.position.y = -0.3;
    pregrasp_poser.position.z = 1.3;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pregrasp_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;   // resolution of interpolation
    const double jump_threshold = 0.0; // disable jump threshold
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // if (fraction < 0.9) {
    //     RCLCPP_WARN(LOGGER, "Cartesian path only planned %.2f%% of the way", fraction * 100.0);
    // } else {
    //     RCLCPP_INFO(LOGGER, "Cartesian path planned successfully, %.2f%% achieved", fraction * 100.0);
    //     rclcpp::Rate rate(10);
    //     //RCLCPP_INFO(LOGGER, "Trajectory has %zu points", trajectory.joint_trajectory.points.size());
    //     for (size_t i = 0; i < trajectory.joint_trajectory.points.size() && rclcpp::ok(); ++i) {
    //         std_msgs::msg::Float64MultiArray msg;
    //         msg.data = trajectory.joint_trajectory.points[i].positions;
    //         traj_pub->publish(msg);
    //         // std::ostringstream oss;
    //         // oss << "Published point " << i << ": ";
    //         // for (size_t j = 0; j < msg.data.size(); ++j) {
    //         //     oss << trajectory.joint_trajectory.joint_names[j] << "=" << msg.data[j] << " ";
    //         // }
    //         // RCLCPP_INFO(LOGGER, "%s", oss.str().c_str());
    //         rate.sleep();
    //     }
    // }
    RCLCPP_INFO(LOGGER, "Cartesian path planned successfully, %.2f%% achieved", fraction * 100.0);
        rclcpp::Rate rate(10);
        //RCLCPP_INFO(LOGGER, "Trajectory has %zu points", trajectory.joint_trajectory.points.size());
        for (size_t i = 0; i < trajectory.joint_trajectory.points.size() && rclcpp::ok(); ++i) {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = trajectory.joint_trajectory.points[i].positions;
            traj_pub->publish(msg);
            rate.sleep();}

    rclcpp::shutdown();
    return 0;
}