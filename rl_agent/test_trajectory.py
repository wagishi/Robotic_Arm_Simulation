import rclpy
from rclpy.node import Node
from trajectory_client import TrajectoryClient

def main():
    rclpy.init()
    node = Node('test_trajectory_client')

    joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
    client = TrajectoryClient(node, joint_names)

    target = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.get_logger().info('Sending test goal...')
    success = client.send_goal(target, time_from_start_sec=3.0)
    node.get_logger().info(f'Motion succeeded: {success}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
