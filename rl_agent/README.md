# RL Agent Module

Reinforcement learning module for arm manipulation (pick-and-place fine control).

## Status
- `trajectory_client.py` — ROS2 action client wrapping FollowJointTrajectory. Working.
- `test_trajectory.py` — standalone test script to verify arm motion via the client.
- Currently debugging a Gazebo simulation timing issue (trajectory goals accepted but execution times out) before proceeding to the Gymnasium environment (`arm_env.py`).

## Next steps
- Fix simulation timeout
- Build `robot_state.py` (joint state + end-effector pose reader)
- Build `arm_env.py` (Gymnasium environment)
- Train PPO agent with Stable-Baselines3
