"""Standalone PiPuck waypoint-navigation and obstacle-avoidance experiment.

The robot uses wheel-encoder odometry to approach a fixed (x, y) target.
On detecting a front obstacle, it rotates until the right-side sensors
see a nearby boundary, then follows that boundary until it clears.

This is a reactive, open-ended experiment, not the R&B/Wi-Fi team mission.
It does not provide collision-free navigation or a recovery timeout.
"""

from pipuck import PiPuck
from time import sleep, time
import math
from experiments.odometry import Odometry

class StateMachine:
    """Navigate to a fixed waypoint using SEEK, AVOID, and STOP states.

    The controller uses PiPuck proximity readings for reactive obstacle
    avoidance and an external Odometry instance to track estimated pose.
    """

    def __init__(self, puck):
        """Initialize robot access, waypoint, speed, and boundary-control gains.

        Args:
            puck: Initialized PiPuck interface providing ``epuck2`` commands.
        """
        self.puck = puck
        self.state = "SEEK"
        self.speed = 210
        self.odom = Odometry()
        # Fixed waypoint in the odometry coordinate frame.
        self.target_x = 200.0
        self.target_y = 400.0
        # Right-wall following uses P and D terms (no integral term).
        self.target_prox = 150
        self.kp = 0.05
        self.kd = 0.12
        self.previous_error = 0

    def stop_motors(self):
        """Command zero wheel speeds and transmit the update to the robot."""
        print("Stopping motors...")
        self.puck.epuck2.set_wheel_speeds(0, 0)
        self.puck.epuck2.update()

    def run_mission(self):
        """Run the sensor-update and navigation loop until STOP.

        In SEEK, a front obstacle triggers a blocking left pivot until the
        front clears and the right-side sensors detect a boundary. The
        controller then enters AVOID. A keyboard interrupt is handled by
        the standalone entry point, not by this method.
        """
        print(f"Mission started! Target: ({self.target_x}, {self.target_y})")
        while True:
            frame = self.puck.epuck2.update()
            curr_x, curr_y, curr_theta = self.odom.update(*frame.motor_steps)
            prox = list(frame.prox)
            front_val = max(prox[0], prox[7], prox[1], prox[6])

            # Front obstacle detection takes precedence over waypoint steering.
            if self.state == "SEEK" and front_val > 200:
                print("Obstacle! Positioning...")
                while True:
                    self.puck.epuck2.set_wheel_speeds(-120, 120)
                    f = self.puck.epuck2.update()
                    self.odom.update(*f.motor_steps)
                    p = list(f.prox)
                    if max(p[0], p[7]) < 200 and max(p[2], p[3]) > 150:
                        break
                    sleep(0.02)
                self.state = "AVOID"
                continue

            if self.state == "SEEK":
                self._state_seek(curr_x, curr_y, curr_theta)
            elif self.state == "AVOID":
                self._state_avoid(curr_x, curr_y, curr_theta)
            elif self.state == "STOP":
                print(f"Goal Reached! Final Pos: ({curr_x:.1f}, {curr_y:.1f})")
                self.stop_motors()
                break
            sleep(0.05)

    def _state_seek(self, curr_x, curr_y, curr_theta):
        """Steer toward the waypoint and enter STOP when sufficiently close.

        Args:
            curr_x: Current odometry x coordinate.
            curr_y: Current odometry y coordinate.
            curr_theta: Estimated heading in degrees.
        """
        dx, dy = self.target_x - curr_x, self.target_y - curr_y
        distance = math.sqrt(dx**2 + dy**2)
        if distance < 10.0:
            self.state = "STOP"
            return

        # The heading convention measures angles from the positive y axis.
        target_angle = math.degrees(math.atan2(dx, dy))
        angle_error = (target_angle - curr_theta + 180) % 360 - 180
        if -15 < angle_error < 15:
            self.puck.epuck2.set_wheel_speeds(self.speed, self.speed)
        elif angle_error >= 15:
            self.puck.epuck2.set_wheel_speeds(self.speed, int(self.speed * 0.4))
        else:
            self.puck.epuck2.set_wheel_speeds(int(self.speed * 0.4), self.speed)

    def _state_avoid(self, curr_x, curr_y, curr_theta):
        """Follow a right-side obstacle boundary with a P/D correction.

        This method reads a fresh proximity frame. The pose arguments are
        currently unused; control depends on right-side proximity only.

        Args:
            curr_x: Current odometry x coordinate (unused).
            curr_y: Current odometry y coordinate (unused).
            curr_theta: Current heading (unused).
        """
        frame = self.puck.epuck2.update()
        prox = list(frame.prox)
        right_val = max(prox[2], prox[3])

        if right_val < 100:
            print(f"Obstacle cleared. Resuming SEEK.")
            self.state = "SEEK"
            return

        error = right_val - self.target_prox
        # Difference between readings, not a time-normalized derivative.
        derivative = error - self.previous_error
        correction = (error * self.kp) + (derivative * self.kd)
        self.puck.epuck2.set_wheel_speeds(int(self.speed - correction), int(self.speed + correction))
        self.previous_error = error

    def cleanup(self):
        """Stop both motors when the experiment exits."""
        self.stop_motors()

if __name__ == "__main__":
    puck = PiPuck()
    brain = None
    # Keep the robot connection active throughout the experiment.
    with puck:
        try:
            brain = StateMachine(puck)
            brain.run_mission()
        except KeyboardInterrupt:
            print("\n[INTERRUPT] Stopping robot safely...")
        finally:
            # Stop the motors whether navigation completes or raises an error.
            if brain:
                brain.cleanup()
            else:
                # Handle failures before the controller was constructed.
                puck.epuck2.set_wheel_speeds(0, 0)
                puck.epuck2.update()
