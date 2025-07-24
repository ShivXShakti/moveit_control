#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <chrono>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class PlannedJointExtractor : public rclcpp::Node
{
public:
    PlannedJointExtractor() : Node("planned_joint_extractor") {
        joint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    }

    void run()
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for robot_description...");
        rclcpp::sleep_for(500ms);

        auto move_group = std::make_shared<MoveGroupInterface>(shared_from_this(), "urs_arml");

        // Define target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.3;
        target_pose.position.y = 0.3;
        target_pose.position.z = 1.3;
        target_pose.orientation.w = 0.6533;
        target_pose.orientation.x = 0.6533;
        target_pose.orientation.y = 0.2706;
        target_pose.orientation.z = 0.2706;

        

        move_group->setPoseTarget(target_pose);

        // Plan to the pose
        MoveGroupInterface::Plan my_plan;
        auto success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful!");

            const auto& points = my_plan.trajectory_.joint_trajectory.points;

            rclcpp::Rate rate(100); // 100 Hz

            for (size_t i = 0; i < points.size() && rclcpp::ok(); ++i)
            {
                const auto& point = points[i];

                std_msgs::msg::Float64MultiArray msg;
                msg.data = point.positions;  // directly copy 6 joint positions

                joint_pub_->publish(msg);

                std::ostringstream joint_str;
                joint_str << "Published joint values [";
                for (size_t j = 0; j < point.positions.size(); ++j)
                {
                    joint_str << point.positions[j];
                    if (j != point.positions.size() - 1) joint_str << ", ";
                }
                joint_str << "]";
                RCLCPP_INFO(this->get_logger(), "%s", joint_str.str().c_str());

                rate.sleep();
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannedJointExtractor>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
