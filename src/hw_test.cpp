#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <darm_msgs/msg/ui_command.hpp>
#include <darm_msgs/msg/ui_status.hpp>

#include <vector>
#include <chrono>
#include <thread>

class DarmPositionController : public rclcpp::Node
{
public:
    DarmPositionController()
        : Node("darm_position_controller")
    {
        pub_ = this->create_publisher<darm_msgs::msg::UiCommand>(
            "/svaya/ui/command", 10);

        sub_ = this->create_subscription<darm_msgs::msg::UiStatus>(
            "/svaya/ui/status", 10,
            std::bind(&DarmPositionController::statusCallback, this, std::placeholders::_1));

        initTrajectory();
    }

    void run()
    {
        //initializeCurrentPosition();
        //waitForDeveloperMode();
        executeTrajectory();
    }

private:
    /* ---------- ROS ---------- */
    rclcpp::Publisher<darm_msgs::msg::UiCommand>::SharedPtr pub_;
    rclcpp::Subscription<darm_msgs::msg::UiStatus>::SharedPtr sub_;

    darm_msgs::msg::UiStatus::SharedPtr joint_msg_;
    bool status_received_ = false;

    /* ---------- Trajectory ---------- */
    Eigen::MatrixXd trajectory_;   // rows x 7
    Eigen::VectorXd current_pos_{16};
    Eigen::VectorXd target_pos_{16};
    Eigen::VectorXd target_vel_{16};

    /* ---------- Callback ---------- */
    void statusCallback(const darm_msgs::msg::UiStatus::SharedPtr msg)
    {
        joint_msg_ = msg;
        status_received_ = true;
    }

    /* ---------- Manual Trajectory ---------- */
    void initTrajectory()
    {
        std::vector<std::vector<double>> user_values = {
            {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7},
            {0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75},
            {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8}
        };

        trajectory_ = Eigen::MatrixXd(user_values.size(), 7);
        for (size_t i = 0; i < user_values.size(); ++i)
            for (size_t j = 0; j < 7; ++j)
                trajectory_(i, j) = user_values[i][j];
    }

    /* ---------- Helpers ---------- */
    void waitForFirstStatus()
    {
        while (rclcpp::ok() && !status_received_)
        {
            rclcpp::spin_some(shared_from_this());
            RCLCPP_INFO(this->get_logger(), "Waiting for UI status...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void initializeCurrentPosition()
    {
        for (int i = 0; i < 7; i++)
        {
            current_pos_[i]     = joint_msg_->left_arm.position[i];
            current_pos_[i + 7] = joint_msg_->right_arm.position[i];

            if (i < 2)
                current_pos_[i + 14] = joint_msg_->head.position[i];
        }
        target_pos_ = current_pos_;
    }

    /* ---------- Trajectory Execution ---------- */
    void executeTrajectory()
    {
        darm_msgs::msg::UiCommand cmd;
        cmd.developer_command.enable = true;
        cmd.developer_command.command.resize(16);

        RCLCPP_INFO(this->get_logger(), "Joint Trajectory Started");

        for (int row = 0; rclcpp::ok() && row < trajectory_.rows(); row++)
        {
            rclcpp::spin_some(shared_from_this());

            for (int i = 0; i < 7; i++)
            {
                // Head
                if (i < 2)
                {
                    target_pos_[i + 14] = 0.0;//joint_msg_->head.position[i];
                    target_vel_[i + 14] = 0.0;//joint_msg_->head.velocity[i];
                }

                // Right arm stays same
                target_pos_[i + 7] = 0.0;//joint_msg_->right_arm.position[i];
                target_vel_[i + 7] = 0.0;//joint_msg_->right_arm.velocity[i];

                // Left arm from trajectory
                target_pos_[i] = 0.0;
                target_vel_[i] = 0.0;//joint_msg_->left_arm.velocity[i];

                cmd.developer_command.command[i].position = target_pos_[i];
                cmd.developer_command.command[i].velocity = target_vel_[i];
            }

            pub_->publish(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        RCLCPP_INFO(this->get_logger(), "Joint Trajectory Completed");
    }
};

/* ---------- main ---------- */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DarmPositionController>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
