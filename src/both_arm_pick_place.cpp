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
static const std::string PLANNING_GROUP = "right_arm";   // <-- must exist in SRDF

int main(int argc, char **argv) {
    /*-------------------------------------------------------------------
                                Do not modify
    -----------------------------------------------------------------------*/
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("pick_place_dual", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    auto traj_pub = move_group_node->create_publisher<std_msgs::msg::Float64MultiArray>("planned_trajectory", 10);
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),
              move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));
    
    /*-------------------------------------------------------------------
                                MODIFY
    -----------------------------------------------------------------------*/
    move_group.setPlanningTime(30.0);
    move_group.setGoalOrientationTolerance(0.1);  // ~6 degrees

    // moveit::planning_interface::MoveGroupInterface left_group(move_group_node, "left_arm");
    // RCLCPP_INFO(LOGGER, "Left EE: %s", left_group.getEndEffectorLink().c_str());
    // moveit::planning_interface::MoveGroupInterface right_group(move_group_node, "right_arm");
    // RCLCPP_INFO(LOGGER, "Right EE: %s", right_group.getEndEffectorLink().c_str());


    
    /*-------------------------------------------------------------------
                                DEFINE POSES FOR BOTH ARMS
    -----------------------------------------------------------------------*/
    //std::vector<geometry_msgs::msg::Pose> left_poses;
    std::vector<geometry_msgs::msg::Pose> right_poses;

    // Pose 1
    geometry_msgs::msg::Pose r1;//,l1;
    // l1.orientation = rpyToQuat(50, 0, 0); 
    // l1.position.x = 0.5; 
    // l1.position.y = 0.3; 
    // l1.position.z = 1.3;
    r1.orientation = rpyToQuat(50, 0, 0);
    r1.position.x = 0.5; 
    r1.position.y = -0.3; 
    r1.position.z = 1.3;
    //left_poses.push_back(l1);
    right_poses.push_back(r1);

    // // Pose 2
    //geometry_msgs::msg::Pose l2;//, r2;
    //l2.orientation = rpyToQuat(0, 0, 0);
    //l2.position.x = 0.5; l2.position.y = 0.3; l2.position.z = 1.5;
    // r2.orientation = rpyToQuat(0, 0, 0);
    // r2.position.x = 0.6; r2.position.y = -0.4; r2.position.z = 1.0;
    //left_poses.push_back(l2);
    // right_poses.push_back(r2);

    /*-------------------------------------------------------------------
                                PLAN AND EXECUTE
    -----------------------------------------------------------------------*/
    // Iterate over poses
    for (size_t i = 0; i < right_poses.size(); ++i) {
        //move_group.setPoseTarget(left_poses[i], "L_tool0");
        move_group.setPoseTarget(right_poses[i], "R_tool0");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
        RCLCPP_INFO(LOGGER, "Dual-arm plan successful!");
        // Publish trajectory
        rclcpp::Rate rate(10);
        for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size() && rclcpp::ok(); ++i) {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = plan.trajectory_.joint_trajectory.points[i].positions;
            traj_pub->publish(msg);
            rate.sleep();
        }
        } else {
            RCLCPP_ERROR(LOGGER, "Planning for both arms failed.");
        }
    }


    rclcpp::shutdown();
    return 0;
}
