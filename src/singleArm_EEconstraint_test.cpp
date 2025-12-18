#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <sstream>
#include <cmath>
#include <iterator>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

#include <dualarm_custom_msgs/msg/obj_pose_array.hpp>
#include <dualarm_custom_msgs/msg/obj_pose.hpp>

#include <darm_msgs/msg/ui_command.hpp>
#include <darm_msgs/msg/ui_status.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class MoveItPlanner : public rclcpp::Node{
public:
    MoveItPlanner(): Node("pick_place"),planning_group_("arm_left")
    {
        pub_hw = this->create_publisher<darm_msgs::msg::UiCommand>("/svaya/ui/command", 10);
        pub_hw_freq = 20.0;
    }
    
    void setupMoveGroup()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), planning_group_);
        move_group_->setPoseReferenceFrame("torso_hw");
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
        std::copy(move_group_->getJointModelGroupNames().begin(),
                  move_group_->getJointModelGroupNames().end(),
                  std::ostream_iterator<std::string>(std::cout, ", "));
        
        move_group_->setPlanningTime(10.0);
        //set planner
        move_group_->setPlannerId("CHOMP");
        //move_group_->setPlannerId("RRTConnectkConfigDefault");  //ompl
        //move_group_->setNumPlanningAttempts(5);
        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        
        move_group_gripper_left_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->rclcpp::Node::shared_from_this(), "gripper_left");

        //current ee pose
        auto current_pose_stamped = move_group_->getCurrentPose();
        auto pose = current_pose_stamped.pose;

        RCLCPP_INFO(this->get_logger(), "Current EE Pose:");
        RCLCPP_INFO(this->get_logger(), "Position: [x: %f, y: %f, z: %f]",
                    pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: [x: %f, y: %f, z: %f, w: %f]",
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w);
    }

    void run()
    {
        if (!rclcpp::ok()) return;

        RCLCPP_INFO(this->get_logger(), "First pose received, starting trajectory planning...");
        /* ==============================
            1. MOVE TO GRASP POSE
           ==============================*/
        geometry_msgs::msg::Pose grasp_pose;
        grasp_pose.orientation.x = 0.5;
        grasp_pose.orientation.y = 0.5;
        grasp_pose.orientation.z = 0.5;
        grasp_pose.orientation.w = 0.5;
        grasp_pose.position.x = 0.51;
        grasp_pose.position.y = 0.2;
        grasp_pose.position.z = -0.4;
        move_group_->setPoseTarget(grasp_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_grasp;
        auto success_grasp = (move_group_->plan(plan_grasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success_grasp) {
            move_group_->execute(plan_grasp);
            //execute_hw(plan_grasp);
            RCLCPP_INFO(this->get_logger(), "========Reached grasp pose==========.");}
        else{
            RCLCPP_INFO(this->get_logger(), "Did not plan trajectory.");
            return;
        }
    }

private:
    double deg2rad(double deg) { return deg * M_PI / 180.0; }
    void setEEConstraints(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &move_group, double tolerance=0.01, double weight=1.0){
        if (!move_group) {
            throw std::runtime_error("MoveGroupInterface pointer is null");
        }

        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = move_group->getEndEffectorLink();
        ocm.header.frame_id = move_group->getPlanningFrame();
        auto current_pose = move_group->getCurrentPose().pose;
        tf2::Quaternion q;
        tf2::fromMsg(current_pose.orientation, q);
        q.normalize();
        ocm.orientation = tf2::toMsg(q);
        ocm.absolute_x_axis_tolerance = tolerance;
        ocm.absolute_y_axis_tolerance = tolerance;
        ocm.absolute_z_axis_tolerance = tolerance;
        ocm.weight = weight;
        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);
        move_group->setPathConstraints(constraints);
    }

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

    std::string planning_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_left_;

    rclcpp::Subscription<dualarm_custom_msgs::msg::ObjPoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<darm_msgs::msg::UiCommand>::SharedPtr pub_hw;

    geometry_msgs::msg::Pose first_pose_;
    bool pose_received_;
    float pub_hw_freq;
};


// ---------------- DRIVER FUNCTION ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MoveItPlanner>();
    node->setupMoveGroup();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    node->run();

    rclcpp::shutdown();
    return 0;
}
