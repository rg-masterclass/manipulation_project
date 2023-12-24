#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = false;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = false;

  // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("gripper_close");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Home
  //   RCLCPP_INFO(LOGGER, "Home Position");

  //   geometry_msgs::msg::Pose home_pose;
  //   home_pose.orientation.x = 0.635;
  //   home_pose.orientation.y = -0.629;
  //   home_pose.orientation.z = 0.310;
  //   home_pose.orientation.w = -0.323;
  //   home_pose.position.x = 0.045;
  //   home_pose.position.y = 0.133;
  //   home_pose.position.z = 0.490;

  //   move_group_arm.setPoseTarget(home_pose);

  //   success_arm = (move_group_arm.plan(my_plan_arm) ==
  //                  moveit::core::MoveItErrorCode::SUCCESS);

  //   move_group_arm.execute(my_plan_arm);

  // - Translation: [0.344, -0.020, 0.244]
  // - Rotation: in Quaternion [0.735, -0.678, 0.004, -0.017]
  // - Rotation: in RPY (radian) [-3.111, 0.018, -1.489]
  // - Rotation: in RPY (degree) [-178.257, 1.051, -85.332]
  // - Matrix:
  // 0.081 -0.996  0.029  0.344
  // -0.997 -0.081  0.021 -0.020
  // -0.018 -0.030 -0.999  0.244
  // 0.000  0.000  0.000  1.000

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 1.000;
  target_pose1.orientation.y = 0.000;
  target_pose1.orientation.z = 0.000;
  target_pose1.orientation.w = 0.000;
  target_pose1.position.x = 0.343; // sim: 0.34;
  target_pose1.position.y = 0.132; // sim: -0.02;
  target_pose1.position.z = 0.244;
  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("gripper_open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.02;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("gripper_close");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Retreat

  //   RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;

  target_pose1.position.z += 0.04;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.04;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

  // Turnover
  RCLCPP_INFO(LOGGER, "Turnover");

  target_pose1.orientation.x = 1.000;
  target_pose1.orientation.y = 0.000;
  target_pose1.orientation.z = 0.000;
  target_pose1.orientation.w = 0.000;
  target_pose1.position.x = -0.5; // 3
  target_pose1.position.y = -0.02;
  target_pose1.position.z = 0.284;
  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("gripper_open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  rclcpp::shutdown();
  return 0;
}
