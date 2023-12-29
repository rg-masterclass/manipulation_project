#include <cmath>
#include <cstdlib>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "grasping_msgs/action/find_graspable_objects.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

using Find = grasping_msgs::action::FindGraspableObjects;
using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {}

void feedback_callback(GoalHandleFind::SharedPtr,
                       const std::shared_ptr<const Find::Feedback> feedback) {
  RCLCPP_INFO(LOGGER, "Ignoring feedback...");
}

void result_callback(const GoalHandleFind::WrappedResult &result) {}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto find_objects_action_client = rclcpp_action::create_client<Find>(
      move_group_node->get_node_base_interface(),
      move_group_node->get_node_graph_interface(),
      move_group_node->get_node_logging_interface(),
      move_group_node->get_node_waitables_interface(), "find_objects");

  if (!find_objects_action_client) {
    RCLCPP_ERROR(LOGGER, "Action client not initialized");
    exit(-1);
  }

  if (!find_objects_action_client->wait_for_action_server(
          std::chrono::seconds(10))) {
    RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
    exit(-1);
  }

  auto goal_msg = Find::Goal();
  goal_msg.plan_grasps = false;

  RCLCPP_INFO(LOGGER, "Sending goal...");

  auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();

  send_goal_options.goal_response_callback = &goal_response_callback;
  send_goal_options.feedback_callback = &feedback_callback;
  send_goal_options.result_callback = &result_callback;

  auto goal_handle_future =
      find_objects_action_client->async_send_goal(goal_msg, send_goal_options);

  auto result =
      find_objects_action_client->async_get_result(goal_handle_future.get());

  while (result.get().code == rclcpp_action::ResultCode::UNKNOWN) {
    rclcpp::spin_some(move_group_node);
  }

  float dim_x, dim_y, dim_z;
  float pos_x, pos_y, pos_z;

  float grab_x, grab_y, grab_z;

  if (result.get().code == rclcpp_action::ResultCode::SUCCEEDED &&
      sizeof(result.get().result->objects) > 0) {

    auto object = result.get().result->objects[0];

    // RCLCPP_INFO(LOGGER, "Type: %d", object.object.primitives[0].type);
    // RCLCPP_INFO(LOGGER, "DIM X: %f", );
    // RCLCPP_INFO(LOGGER, "DIM Y: %f",
    // object.object.primitives[0].dimensions[1]); RCLCPP_INFO(LOGGER, "DIM Z:
    // %f", object.object.primitives[0].dimensions[2]);

    // RCLCPP_INFO(LOGGER, "POS X: %f",
    //             object.object.primitive_poses[0].position.x);
    // RCLCPP_INFO(LOGGER, "POS Y: %f",
    //             object.object.primitive_poses[0].position.y);
    // RCLCPP_INFO(LOGGER, "POS Z: %f",
    //             object.object.primitive_poses[0].position.z);

    dim_x = object.object.primitives[0].dimensions[0];
    dim_y = object.object.primitives[0].dimensions[1];
    dim_z = object.object.primitives[0].dimensions[2];

    pos_x = object.object.primitive_poses[0].position.x;
    pos_y = object.object.primitive_poses[0].position.y;
    pos_z = object.object.primitive_poses[0].position.z;

    grab_x = std::abs(pos_x) + (dim_x / 2);
    grab_y = std::abs(pos_y) * 2;

    if (pos_x < 0) {
      grab_x = grab_x * -1;
    }

    if (pos_y < 0) {
      grab_y = grab_y * -1;
    }

    grab_x = std::ceil(grab_x * 100.0) / 100.0;
    grab_y = std::ceil(grab_y * 100.0) / 100.0;

    RCLCPP_INFO(LOGGER, "GRAB X: %f", grab_x);
    RCLCPP_INFO(LOGGER, "GRAB Y: %f", grab_y);

  } else {
    RCLCPP_ERROR(LOGGER, "Action call has been canceled or aborted.");
    exit(-1);
  }

  //   RCLCPP_INFO(LOGGER, "Exit before the manipulation part! Remove it!");
  //   exit(1);

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

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 1.000;
  target_pose1.orientation.y = 0.000;
  target_pose1.orientation.z = 0.000;
  target_pose1.orientation.w = 0.000;
  target_pose1.position.x = grab_x; // 0.34; // 3
  target_pose1.position.y = grab_y; // -0.02;
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
