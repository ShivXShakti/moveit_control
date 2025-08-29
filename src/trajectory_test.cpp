#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "left_arm";

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node =
        rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
    
        rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node,
    PLANNING_GROUP);
    
    const moveit::core::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    RCLCPP_INFO(LOGGER, "Planning frame: %s",
    
        move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s",
    move_group.getEndEffectorLink().c_str());
    
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),
    move_group.getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(std::cout, ", "));
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,
    joint_group_positions);
    
    RCLCPP_INFO(LOGGER, "Current joint positions:");
    for (std::size_t i = 0; i < joint_group_positions.size(); ++i) {
    RCLCPP_INFO(LOGGER, "  Joint %zu: %f", i, joint_group_positions[i]);
    }
    joint_group_positions[0] = 0.1; 
    joint_group_positions[1] = -0.10; 
    joint_group_positions[2] = 0.10;
    joint_group_positions[3] = -0.10;
    joint_group_positions[4] = -0.10; 
    joint_group_positions[5] = 0.010; 
    joint_group_positions[6] = 0.100;
    move_group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success =
    (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

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