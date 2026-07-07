"""
Minimal action client for commanding the Kinova arm via FollowJointTrajectory.
This is the ONLY file that talks to the robot's motion controller.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TrajectoryClient:
    def __init__(self, node: Node, joint_names: list,
                 action_name: str = '/joint_trajectory_controller/follow_joint_trajectory'):
        self.node = node
        self.joint_names = joint_names

        self._client = ActionClient(node, FollowJointTrajectory, action_name)

        self.node.get_logger().info(f'Waiting for action server: {action_name}')
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'Action server {action_name} not available')
        self.node.get_logger().info('Action server ready.')

    def send_goal(self, target_positions: list, time_from_start_sec: float = 2.0,
                  timeout_sec: float = 10.0) -> bool:
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = Duration(sec=int(time_from_start_sec),
                                          nanosec=int((time_from_start_sec % 1) * 1e9))
        goal_msg.trajectory.points = [point]

        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().warn('Goal rejected by controller')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)

        result = result_future.result()
        if result is None:
            self.node.get_logger().warn('Trajectory execution timed out')
            return False

        success = (result.result.error_code == 0)
        if not success:
            self.node.get_logger().warn(f'Trajectory failed, error_code={result.result.error_code}')
        return success
