# 🤖 PiPuck Robotics — Early Experiments

This repository contains early robotics experiments developed using the **PiPuck (e-puck2)** platform. The project explores autonomous navigation, obstacle avoidance, infrared (IR) proximity sensing, and cooperative behavior between robots.

## 📁 Project Files

| File | Description |
|---|---|
| `Tri-Walker.py` | An early standalone robot movement experiment. |
| `follow-wall.py` | Wall-following algorithm using IR proximity sensors. The robot detects a wall, aligns itself, and follows its boundary using feedback-based wheel control. |
| `run_away.py` | A standalone robot avoidance experiment. |
| `reach_goal.py` | Autonomous navigation toward a predefined target using wheel odometry and IR obstacle detection. |
| `follow_obj.py` | Follower controller that uses IR proximity sensors to detect and follow another robot, adjusting speed and direction to maintain relative positioning. |
| `team_mission.py` | Coordinates a two-robot leader–follower mission, including Bluetooth communication, leader election, and mission synchronization. |
| `setup.py` | Project setup script. |

## Leader–Follower Mission

The main cooperative experiment combines three files:

- **`team_mission.py`** — Coordinates the robots over Bluetooth, elects their roles, and synchronizes mission startup.
- **`reach_goal.py`** — Controls the leader, which navigates toward a target while avoiding obstacles.
- **`follow_obj.py`** — Controls the follower, which uses IR proximity measurements to track the leader and adjust its movement.

### How it works

1. Two PiPuck robots establish a Bluetooth connection.
2. A leader election determines which robot becomes the leader and which becomes the follower.
3. The follower locates the leader using its IR proximity sensors and signals readiness.
4. The leader begins navigating toward its target.
5. The follower adjusts its wheel speeds to maintain its position relative to the leader.

**Note:** This experiment uses IR proximity sensing for following and Bluetooth for coordination. It is separate from the newer Range & Bearing (R&B) mission.

## 🛠️ Technologies

- Python 3
- PiPuck / e-puck2 robotics platform
- Infrared proximity sensors
- Bluetooth communication
- Wheel-encoder odometry
- Feedback-based motion control

## 🎯 Project Purpose

These experiments explore fundamental concepts in mobile robotics and provide a foundation for developing more advanced cooperative multi-robot
