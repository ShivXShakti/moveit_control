#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <darm_msgs/msg/ui_command.hpp>
#include <darm_msgs/msg/ui_status.hpp>

using namespace std::chrono_literals;

class MoveItPlanner : public rclcpp::Node {
public:
    MoveItPlanner() : Node("dual_arm_planner") {
        traj_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("planned_trajectory", 10);
        pub_hw = this->create_publisher<darm_msgs::msg::UiCommand>("/svaya/ui/command", 10);
        sub_hw = this->create_subscription<darm_msgs::msg::UiStatus>("/svaya/ui/status", 10, 
            std::bind(&MoveItPlanner::callback, this, std::placeholders::_1)); 
        pub_hw_freq = 10.0;
    }

    void setupMoveGroups()
    {
        move_group_left_  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_left");
    
        move_group_gripper_left_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), "gripper_left");

        move_group_left_->setPlanningTime(10.0);
        move_group_left_->setPoseReferenceFrame("torso_hw");
    
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Starting dual-arm trajectory planning…");
        /* ==============================
            1. MOVE TO GRASP POSE
           ==============================*/
        geometry_msgs::msg::Pose pose_left;

        pose_left.orientation.x = 0.5;
        pose_left.orientation.y = 0.5;
        pose_left.orientation.z = 0.5;
        pose_left.orientation.w = 0.5;
        pose_left.position.x = 0.5;
        pose_left.position.y = 0.2;
        pose_left.position.z = -0.4;

        move_group_left_->setPoseTarget(pose_left);
        moveit::planning_interface::MoveGroupInterface::Plan plan_left;
        bool ok_l = (move_group_left_->plan(plan_left) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        if (ok_l) {
            execute_hw(plan_left);
        }else{
            RCLCPP_ERROR(this->get_logger(), "Left arm IK failed!");
            return;
        }

        //auto left_joints = plan_left.trajectory_.joint_trajectory.points.back().positions;
    }

private:

    void execute_hw(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        rclcpp::Rate rate(pub_hw_freq);  // 10 Hz streaming

        auto msg = darm_msgs::msg::UiCommand();
        msg.developer_command.enable = true;

        const auto &traj = plan.trajectory_.joint_trajectory;
        const size_t num_joints = traj.joint_names.size();

        msg.developer_command.command.resize(2*num_joints+2);

        for (size_t i = 0; i < traj.points.size() && rclcpp::ok(); ++i)
        {
            const auto &pt = traj.points[i];

            for (size_t j = 0; j < num_joints; ++j)
            {
                msg.developer_command.command[j].position = pt.positions[j];
                if (!pt.velocities.empty())
                    msg.developer_command.command[j].velocity = pt.velocities[j];
                else
                    msg.developer_command.command[j].velocity = 0.0;
                
                msg.developer_command.command[j+7].position = 0.0;
                msg.developer_command.command[j+7].velocity = 0.0;
                
                if (j < 2){
                    msg.developer_command.command[j+14].position = 0.0;
                    msg.developer_command.command[j+14].velocity = 0.0;}
                }
            pub_hw->publish(msg);
            std::cout << "published joint " << i+1 << ": " << msg.developer_command.command[0].position << ", "<< msg.developer_command.command[1].position << ", "<< msg.developer_command.command[2].position << ", "<< msg.developer_command.command[3].position << ", "<< msg.developer_command.command[4].position << ", "<< msg.developer_command.command[5].position << ", "<< msg.developer_command.command[6].position << ", "
            << msg.developer_command.command[7].position << ", "<< msg.developer_command.command[8].position << ", "<< msg.developer_command.command[9].position << ", "<< msg.developer_command.command[10].position << ", "<< msg.developer_command.command[11].position << ", "<< msg.developer_command.command[12].position << ", "<< msg.developer_command.command[13].position << ", "<< msg.developer_command.command[14].position << ", "<<msg.developer_command.command[15].position << " rad" << std::endl;
            rate.sleep();
        }
    }

    void callback(darm_msgs::msg::UiStatus::SharedPtr msg){
        joint_msg = msg;
        joint_callback_status_ = true;
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_pub_;
    rclcpp::Subscription<darm_msgs::msg::UiStatus>::SharedPtr sub_hw;
    rclcpp::Publisher<darm_msgs::msg::UiCommand>::SharedPtr pub_hw;
    darm_msgs::msg::UiStatus::SharedPtr joint_msg;
    bool joint_callback_status_{false};
    float pub_hw_freq;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_left_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_left_;
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
