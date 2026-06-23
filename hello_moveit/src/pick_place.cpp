#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;



bool commandGripper(
    rclcpp::Node::SharedPtr node,
    double position)
{
    auto client =
        rclcpp_action::create_client<GripperCommand>(
            node,
            "/robotiq_gripper_controller/gripper_cmd");

    if (!client->wait_for_action_server(5s))
    {
        std::cout << "Gripper action server not available"
                  << std::endl;
        return false;
    }

    GripperCommand::Goal goal;

    goal.command.position = position;
    goal.command.max_effort = 50.0;

    auto goal_handle_future =
    client->async_send_goal(goal);

  if (goal_handle_future.wait_for(5s) !=
      std::future_status::ready)
  {
      return false;
  }

  auto goal_handle = goal_handle_future.get();

  if (!goal_handle)
  {
      return false;
  }

  auto result_future =
    client->async_get_result(goal_handle);

  if (result_future.wait_for(10s) !=
      std::future_status::ready)
  {
      return false;
  }

  auto wrapped_result = result_future.get();

  return wrapped_result.code ==
        rclcpp_action::ResultCode::SUCCEEDED;
      }

bool moveToPose(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::Pose& target_pose)
{
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
        (move_group.plan(plan) ==
         moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        std::cout << "Planning failed" << std::endl;
        return false;
    }

    auto result = move_group.execute(plan);

    move_group.clearPoseTargets();

    return result ==
           moveit::core::MoveItErrorCode::SUCCESS;
}


int main(int argc, char * argv[])
{

  constexpr double GRIPPER_OPEN = 0.0;
  constexpr double GRIPPER_CLOSED = 1.0;

  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<rclcpp::Node>(
          "hello_moveit",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("hello_moveit");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  auto failAndExit =
    [&]()
    {
        rclcpp::shutdown();
        spinner.join();
        return 1;
    };

  moveit::planning_interface::MoveGroupInterface move_group(
      node,
      "manipulator");

  RCLCPP_INFO(logger, "MoveGroupInterface initialized");

  move_group.startStateMonitor();

  rclcpp::sleep_for(2s);

  RCLCPP_INFO(logger, "Opening gripper at startup");

  if(!commandGripper(node, GRIPPER_OPEN))
  {
      RCLCPP_ERROR(logger, "Failed to initialize gripper");
      return failAndExit();
  }

auto approach_pose = move_group.getCurrentPose().pose;

RCLCPP_INFO(
    logger,
    "Start pose: x=%.3f y=%.3f z=%.3f",
    approach_pose.position.x,
    approach_pose.position.y,
    approach_pose.position.z);

move_group.setPlannerId("RRTConnectkConfigDefault");
move_group.setPlanningTime(10.0);

move_group.setMaxVelocityScalingFactor(0.3);
move_group.setMaxAccelerationScalingFactor(0.3);

move_group.setGoalPositionTolerance(0.005);
move_group.setGoalOrientationTolerance(0.01);

// ---------- APPROACH ----------
approach_pose.position.z -= 0.05;

RCLCPP_INFO(logger, "Moving to approach pose");

if (!moveToPose(move_group, approach_pose))
{
    RCLCPP_ERROR(logger, "Approach failed");
    return failAndExit();
}

// ---------- GRASP ----------
auto grasp_pose = approach_pose;
grasp_pose.position.z -= 0.05;

RCLCPP_INFO(logger, "Moving to grasp pose");

if (!moveToPose(move_group, grasp_pose))
{
    RCLCPP_ERROR(logger, "Grasp failed");
    return failAndExit();
}

// ---------- CLOSE GRIPPER ----------
RCLCPP_INFO(logger, "Closing gripper");

if(!commandGripper(node, GRIPPER_CLOSED))
{
    RCLCPP_ERROR(logger, "Failed to close gripper");
    return failAndExit();
}

rclcpp::sleep_for(1s);

// ---------- LIFT ----------
auto lift_pose = grasp_pose;
lift_pose.position.z += 0.10;

RCLCPP_INFO(logger, "Moving to lift pose");

if (!moveToPose(move_group, lift_pose))
{
    RCLCPP_ERROR(logger, "Lift failed");
    return failAndExit();
}

// ---------- PLACE ----------
auto place_pose = lift_pose;
place_pose.position.y -= 0.10;

RCLCPP_INFO(logger, "Moving to place pose");

if (!moveToPose(move_group, place_pose))
{
    RCLCPP_ERROR(logger, "Place motion failed");
    return failAndExit();
}

// ---------- OPEN ----------
RCLCPP_INFO(logger, "Opening gripper");

if(!commandGripper(node, GRIPPER_OPEN))
{
    RCLCPP_ERROR(logger, "Failed to open gripper");
    return failAndExit();
}

rclcpp::sleep_for(1s);

// ---------- RETREAT ----------
auto retreat_pose = place_pose;
retreat_pose.position.z += 0.02;
retreat_pose.position.y += 0.05;

RCLCPP_INFO(logger, "Retreating");

if (!moveToPose(move_group, retreat_pose))
{
    RCLCPP_ERROR(logger, "Retreat failed");
    return failAndExit();
}

//   geometry_msgs::msg::Pose approach_pose;
//   approach_pose.orientation.w = 1.0;
//   approach_pose.position.x = 0.30;
//   approach_pose.position.y = 0.20;
//   approach_pose.position.z = 0.50;

//   geometry_msgs::msg::Pose grasp_pose = approach_pose;
//   grasp_pose.position.z -= 0.10;

//   geometry_msgs::msg::Pose lift_pose = grasp_pose;
//   lift_pose.position.z += 0.15;

//   geometry_msgs::msg::Pose place_pose = lift_pose;
//   place_pose.position.y -= 0.30;

//   RCLCPP_INFO(logger, "Moving to approach pose");
//   moveToPose(move_group, approach_pose);

//   RCLCPP_INFO(logger, "Moving to grasp pose");
//   moveToPose(move_group, grasp_pose);

//   RCLCPP_INFO(logger, "CLOSE GRIPPER");

//   auto pose = move_group.getCurrentPose();

// RCLCPP_INFO(
//     logger,
//     "Position: x=%f y=%f z=%f",
//     pose.pose.position.x,
//     pose.pose.position.y,
//     pose.pose.position.z);

// RCLCPP_INFO(
//     logger,
//     "Orientation: x=%f y=%f z=%f w=%f",
//     pose.pose.orientation.x,
//     pose.pose.orientation.y,
//     pose.pose.orientation.z,
//     pose.pose.orientation.w);

//   RCLCPP_INFO(logger, "Moving to lift pose");
//   moveToPose(move_group, lift_pose);

//   RCLCPP_INFO(logger, "Moving to place pose");
//   moveToPose(move_group, place_pose);

//   RCLCPP_INFO(logger, "OPEN GRIPPER");

  auto current_pose = move_group.getCurrentPose();

  RCLCPP_INFO(
      logger,
      "Current position: x=%f y=%f z=%f",
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      current_pose.pose.position.z);

  rclcpp::shutdown();
  spinner.join();

  return 0;
}