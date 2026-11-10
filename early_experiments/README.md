# PiPuck Robotics — Early Experiments

This directory contains early robotics experiments developed using the **PiPuck (e-puck2)** platform. The experiments explore autonomous navigation, obstacle avoidance, infrared (IR) proximity sensing, and cooperative behavior between robots.

## 📁 Project Files

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
5. The follower continuously adjusts its wheel speeds to maintain its relative position behind the leader.

This experiment combines **Bluetooth communication for coordination** with **IR proximity sensors for physical following and obstacle detection**.

### Running the Mission

The repository includes two shell scripts for managing the mission from your computer.

Before starting:

- Make sure both PiPucks are powered on, connected to a reachable network, and accessible over SSH.
- Ensure the three Python mission files are installed together in the working directory expected by the launcher on each robot.
- Check the robot hostnames and remote paths configured in `start_team.sh`.
- Configure non-interactive SSH authentication if required by the launcher.

From your computer, navigate to the `early_experiments` directory:

```bash
cd early_experiments
```

Make the scripts executable if necessary:

```bash
chmod +x start_team.sh stop_team.sh
```

**Start the mission:**

```bash
./start_team.sh
```

The script starts the IR team mission on the configured robots. The robots then perform leader election and execute their respective roles.

**Stop the mission:**

```bash
./stop_team.sh
```

This script stops the team mission processes on the configured robots.

> **Important:** The launcher executes `team_mission.py` from a configured directory on each robot. Since the Python files are stored locally under `ir_mission/`, make sure the launcher and remote file locations are consistent. Moving the files into this folder on GitHub does not automatically move them on the robots.

## Technologies

- Python 3
- PiPuck / e-puck2 robotics platform
- Infrared proximity sensors
- Bluetooth communication
- Wheel-encoder odometry
- Feedback-based motion control
- SSH-based remote execution

## Project Purpose

These early experiments explore fundamental concepts in mobile robotics and provide a foundation for developing more advanced cooperative multi-robot navigation systems.

The IR leader–follower experiment is separate from the newer [Range & Bearing (R&B) team mission](../main_rab_mission/README.md), which uses dedicated R&B sensors and Wi-Fi communication for more advanced multi-robot coordination.
