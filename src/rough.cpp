geometry_msgs::msg::Pose rightGoalPose = msg->poses.back();
  geometry_msgs::msg::Pose leftGoalPose = msg->poses.front();

    // Calculate Kinematics for each arm individually using separate move_groups
    leftArmMoveGroup->setJointValueTarget(leftGoalPose);
    rightArmMoveGroup->setJointValueTarget(rightGoalPose);

    std::vector<double> leftJointValue;
    std::vector<double> rightJointValue;


    leftArmMoveGroup->getJointValueTarget(leftJointValue);
    rightArmMoveGroup->getJointValueTarget(rightJointValue);

    std::vector<double> bothJointsSend;

    bothJointsSend.insert(bothJointsSend.end(), rightJointValue.begin(),
                          rightJointValue.end());
    bothJointsSend.insert(bothJointsSend.end(), leftJointValue.begin(),
                          leftJointValue.end());

    auto successCheck = bothArmsMoveGroup->setJointValueTarget(bothJointsSend);

    std::vector<double> bothJointValue;
    bothArmsMoveGroup->getJointValueTarget(bothJointValue);


    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planning_success =
        (bothArmsMoveGroup->plan(plan) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (planning_success) {
      RCLCPP_INFO(LOGGER, "Planning succeeded. Executing movement...");
      bothArmsMoveGroup->execute(plan);
      // rightArmMoveGroup->execute(planRight);
      RCLCPP_INFO(LOGGER, "Movement executed successfully.");
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed for both arms");
    }