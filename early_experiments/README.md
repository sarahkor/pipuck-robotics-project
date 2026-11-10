# PiPuck Robotics — Early Experiments

This directory contains early robotics experiments developed using the **PiPuck (e-puck2)** platform. The experiments explore autonomous navigation, obstacle avoidance, infrared (IR) proximity sensing, and cooperative behavior between robots.

## 📁 Project Structure

```text
early_experiments/
├── ir_mission/
│   ├── team_mission.py
│   ├── follow_obj.py
│   └── reach_goal.py
├── Tri-Walker.py
├── follow-wall.py
├── run_away.py
├── start_team.sh
├── stop_team.sh
└── README.md
```

## Project Files

| File | Description |
|---|---|
| `Tri-Walker.py` | An early standalone robot movement experiment. |
| `follow-wall.py` | Wall-following algorithm using IR proximity sensors. The robot detects a wall, aligns itself, and follows its boundary using feedback-based wheel control. |
| `run_away.py` | A standalone robot avoidance experiment. |
| `ir_mission/reach_goal.py` | Autonomous navigation toward a predefined target using wheel odometry and IR obstacle detection. |
| `ir_mission/follow_obj.py` | Follower controller that uses IR proximity sensors to detect and follow another robot, adjusting speed and direction to maintain relative positioning. |
| `ir_mission/team_mission.py` | Coordinates the two-robot leader–follower mission, including Bluetooth communication, leader election, and mission synchronization. |
| `start_team.sh` | Launcher script for starting the IR-based leader–follower mission on the robots. |
| `stop_team.sh` | Script for stopping the IR-based team mission processes. |

## IR-Based Leader–Follower Mission

The main cooperative experiment combines three Python files located inside `ir_mission/`:

- **`team_mission.py`** — Coordinates the robots over Bluetooth, elects their roles, and synchronizes mission startup.
- **`reach_goal.py`** — Controls the leader, which navigates toward a target while avoiding obstacles.
- **`follow_obj.py`** — Controls the follower, which uses IR proximity measurements to track the leader and adjust its movement.

### How It Works

1. Two PiPuck robots establish a Bluetooth connection.
2. A leader election determines which robot becomes the leader and which becomes the follower.
3. The follower searches for the leader using its IR proximity sensors and signals readiness.
4. The leader begins navigating toward its target.
