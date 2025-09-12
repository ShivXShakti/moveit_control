cmake_minimum_required(VERSION 3.8)
project(moveit_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(home src/home.cpp)
target_include_directories(home
PUBLIC include)
ament_target_dependencies(home ament_cmake 
  rclcpp rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs)

add_executable(pick_place src/pick_place.cpp)
target_include_directories(pick_place
PUBLIC include)
ament_target_dependencies(pick_place ament_cmake 
  rclcpp rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs)

add_executable(pick_place_main src/pick_place_main.cpp)
target_include_directories(pick_place_main
PUBLIC include)
ament_target_dependencies(pick_place_main ament_cmake 
  rclcpp rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs)

add_executable(both_arm_pick_place src/both_arm_pick_place.cpp)
target_include_directories(both_arm_pick_place
PUBLIC include)
ament_target_dependencies(both_arm_pick_place ament_cmake 
  rclcpp rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs)

add_executable(single_arm_pick_place src/single_arm_pick_place.cpp)
target_include_directories(single_arm_pick_place
PUBLIC include)
ament_target_dependencies(single_arm_pick_place ament_cmake 
  rclcpp rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs)


add_executable(planned_joint_extractor src/planned_joint_extractor.cpp)
target_include_directories(planned_joint_extractor
PUBLIC include)
ament_target_dependencies(planned_joint_extractor ament_cmake 
  rclcpp rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs)

add_executable(trajectory_test src/trajectory_test.cpp)
target_include_directories(trajectory_test
PUBLIC include)
ament_target_dependencies(trajectory_test ament_cmake 
  rclcpp rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs)

add_executable(traj_ee src/traj_ee.cpp)
target_include_directories(traj_ee
PUBLIC include)
ament_target_dependencies(traj_ee ament_cmake 
  rclcpp rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs)

add_executable(traj_cart src/traj_cart.cpp)
target_include_directories(traj_cart
PUBLIC include)
ament_target_dependencies(traj_cart ament_cmake 
  rclcpp rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
interactive_markers
moveit_ros_planning
control_msgs)

install(TARGETS 
    home 
    pick_place_main
    pick_place 
    single_arm_pick_place
    both_arm_pick_place 
    planned_joint_extractor 
    trajectory_test 
    traj_ee 
    traj_cart 
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
