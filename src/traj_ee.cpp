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

/*-------------------------------------------------------------------
                                METHODS
-----------------------------------------------------------------------*/
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "arm"; 


int main(int argc, char **argv) {
  /*-------------------------------------------------------------------
                                Do not modify
    -----------------------------------------------------------------------*/  
  rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("pick_place", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    auto traj_pub = move_group_node->create_publisher<std_msgs::msg::Float64MultiArray>("planned_trajectory", 10);
    // Create MoveGroupInterfaces for left, right and composite arm
moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, "arm");
moveit::planning_interface::MoveGroupInterface move_group_left(move_group_node, "left_arm");
moveit::planning_interface::MoveGroupInterface move_group_right(move_group_node, "right_arm");

// Common planner tuning
move_group_arm.setPlanningTime(8.0);
move_group_arm.setNumPlanningAttempts(8);
move_group_left.setPlanningTime(4.0);
move_group_right.setPlanningTime(4.0);

// Print some debug info
RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
RCLCPP_INFO(LOGGER, "Available Planning Groups:");
std::copy(move_group_arm.getJointModelGroupNames().begin(),
          move_group_arm.getJointModelGroupNames().end(),
          std::ostream_iterator<std::string>(std::cout, ", "));
RCLCPP_INFO(LOGGER, "Note: composite group has empty end-effector link (expected).");

// Define left/right poses (ensure they are different / reachable)
geometry_msgs::msg::Pose left_pose, right_pose;
//left_pose.orientation = rpyToQuat(50.0, 0.0, 0.0);
left_pose.position.x = 0.5;
left_pose.position.y = 0.3;
left_pose.position.z = 1.3;

//right_pose.orientation = rpyToQuat(50.0, 0.0, 0.0);
right_pose.position.x = 0.5;
right_pose.position.y = -0.3; // <- negative y for right
right_pose.position.z = 1.3;

// 1) Quick single-arm checks (helps isolate IK issues)
auto try_single_arm = [&](moveit::planning_interface::MoveGroupInterface &mg,
                          const geometry_msgs::msg::Pose &pose,
                          const std::string &eef_link,
                          const std::string &name) -> bool {
  mg.setStartStateToCurrentState();
  mg.clearPoseTargets();
  mg.setPoseTarget(pose, eef_link);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  RCLCPP_INFO(LOGGER, "[%s] Planning (single-arm) to %s...", name.c_str(), eef_link.c_str());
  bool ok = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (ok) RCLCPP_INFO(LOGGER, "[%s] single-arm plan succeeded (%zu pts).", name.c_str(), plan.trajectory_.joint_trajectory.points.size());
  else RCLCPP_WARN(LOGGER, "[%s] single-arm plan FAILED.", name.c_str());
  return ok;
};

bool left_ok = try_single_arm(move_group_left, left_pose, "L_tool0", "LEFT");
bool right_ok = try_single_arm(move_group_right, right_pose, "R_tool0", "RIGHT");

if (!left_ok || !right_ok) {
  RCLCPP_ERROR(LOGGER, "One or both single-arm plans failed. Inspect poses in RViz or try simpler poses.");
  rclcpp::shutdown();
  return 1;
}

// 2) Now try combined plan (arm group) with both pose targets
move_group_arm.setStartStateToCurrentState();
move_group_arm.clearPoseTargets();
move_group_arm.setPoseTarget(left_pose, "L_tool0");
move_group_arm.setPoseTarget(right_pose, "R_tool0");

// Optional: increase attempts/time for combined planning
move_group_arm.setPlanningTime(12.0);
move_group_arm.setNumPlanningAttempts(12);

moveit::planning_interface::MoveGroupInterface::Plan dual_plan;
RCLCPP_INFO(LOGGER, "Attempting combined dual-arm plan...");
bool success = (move_group_arm.plan(dual_plan) == moveit::core::MoveItErrorCode::SUCCESS);

if (!success) {
  RCLCPP_ERROR(LOGGER, "Combined dual-arm planning FAILED. Try these diagnostics:");
  RCLCPP_ERROR(LOGGER, " - Are targets reachable individually? (we tested above)");
  RCLCPP_ERROR(LOGGER, " - Do the arms collide when both at those poses?");
  RCLCPP_ERROR(LOGGER, " - Try lowering planning difficulty: move targets closer to robot or to 'home'.");
  rclcpp::shutdown();
  return 1;
}

// Publish trajectory like you did before
const auto &traj = dual_plan.trajectory_.joint_trajectory;
RCLCPP_INFO(LOGGER, "Combined plan success — %zu points. Publishing...", traj.points.size());
rclcpp::Rate rate(10);
for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = traj.points[i].positions;
  traj_pub->publish(msg);
  std::ostringstream oss;
  oss << "Published point " << i << ": ";
  for (size_t j = 0; j < msg.data.size(); ++j) oss << dual_plan.trajectory_.joint_trajectory.joint_names[j] << "=" << msg.data[j] << " ";
  RCLCPP_INFO(LOGGER, "%s", oss.str().c_str());
  rate.sleep();
}


    rclcpp::shutdown();
    return 0;
}