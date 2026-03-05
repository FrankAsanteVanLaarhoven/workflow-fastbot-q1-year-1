# Fleet-Safe VLA - FastBot Q1 Year 1

Constrained VLA system for safe multi-robot navigation in hospital environments.

## Stack
- ROS 2 Humble on Ubuntu 22.04
- Nav2 for autonomous navigation
- Isaac Sim for simulation
- FSRL/TorchRL for safe RL
- RTAMT for STL runtime monitoring

## Packages
| Package | Role |
|---|---|
| fastbot_bringup | Launch + baseline nav + data collection |
| fastbot_description | URDF / robot model |
| fastbot_nav2 | Nav2 parameter configs |
| fleetsafe_msgs | Custom ROS 2 messages |
| fleetsafe_monitor | STL safety monitor |
| fleetsafe_rl | Safe RL training (CPO/Lagrangian) |
| fleetsafe_vla | VLA inference node |

## Quick Start
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    ros2 launch fastbot_bringup sim_bringup.launch.py
