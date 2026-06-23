#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
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

  moveit::planning_interface::MoveGroupInterface move_group(
      node,
      "manipulator");

  RCLCPP_INFO(logger, "MoveGroupInterface initialized");

  move_group.startStateMonitor();

    rclcpp::sleep_for(2s);

    geometry_msgs::msg::Pose target_pose;

    target_pose.orientation.w = 1.0;

    target_pose.position.x = 0.30;
    target_pose.position.y = 0.20;
    target_pose.position.z = 0.50;

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
    (move_group.plan(plan) ==
    moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
    RCLCPP_INFO(logger, "Plan successful, executing...");
    move_group.execute(plan);
    }
    else
    {
    RCLCPP_ERROR(logger, "Planning failed");
    }

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