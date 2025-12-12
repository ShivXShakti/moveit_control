#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std::chrono_literals;

class MoveItPlanner : public rclcpp::Node {
public:
    MoveItPlanner() : Node("dual_arm_planner") {}

    void setupMoveGroups()
    {
        move_group_both_  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_both");
        move_group_both_->setPlanningTime(10.0);
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Setting all joint values to ZERO...");

        // -----------------------------------------
        // 1. Get joint list for the combined group
        // -----------------------------------------
        std::vector<std::string> joint_names = move_group_both_->getJointNames();
        size_t n_joints = joint_names.size();

        if (n_joints == 0) {
            RCLCPP_ERROR(this->get_logger(), "No joints found in arm_both!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "arm_both has %zu joints", n_joints);

        // -----------------------------------------
        // 2. Create zero-valued joint vector
        // -----------------------------------------
        std::vector<double> zero_joints(n_joints, 0.0);

        // -----------------------------------------
        // 3. Set joint target
        // -----------------------------------------
        move_group_both_->setJointValueTarget(zero_joints);

        // -----------------------------------------
        // 4. Plan
        // -----------------------------------------
        moveit::planning_interface::MoveGroupInterface::Plan plan_both;
        bool success = (move_group_both_->plan(plan_both) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Planning to ZERO joints failed!");
            return;
        }

        // -----------------------------------------
        // 5. Execute
        // -----------------------------------------
        move_group_both_->execute(plan_both);
        RCLCPP_INFO(this->get_logger(), "Both arms moved to ZERO joint positions!");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_both_;
};

// ------------------ MAIN ------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MoveItPlanner>();
    node->setupMoveGroups();
    node->run();

    rclcpp::shutdown();
    return 0;
}
