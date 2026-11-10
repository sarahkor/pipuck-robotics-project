"""Step-based leader waypoint navigation for the R&B team mission.

The leader uses wheel-encoder odometry to approach a waypoint. A front IR
obstacle interrupts seeking: it pivots left until seven readings fall below
the clear threshold, then turns an additional configured angle before seeking
again. This is reactive clearance, not boundary following or path planning.

Waypoint coordinates are millimeters in the robot's local odometry frame;
heading values returned by Odometry.update() are degrees clockwise from +Y.
"""

import math

from odometry import Odometry


class ReachGoalController:
    """Navigate the team leader to a local waypoint with front-obstacle avoidance.

    Args:
        puck: Active PiPuck hardware interface.
        target_x: Target X coordinate in millimeters, relative to local odometry.
        target_y: Target Y coordinate in millimeters, relative to local odometry.
    """

    def __init__(self, puck, target_x=0.0, target_y=100.0):
        """Initialize the goal, odometry, wheel speed, and obstacle thresholds."""
        self.puck = puck
        self.target_x = float(target_x)
        self.target_y = float(target_y)

        self.odom = Odometry()
        self.last_position = (0.0, 0.0, 0.0)

        self.state = "SEEK"
        self.finished = False

        # The leader must be slower than the follower cruise speed. The mission
        # may reduce this speed further when a follower reports local avoidance.
        self.speed = 150

        self.goal_tolerance_mm = 18.0

        # Enter TURN_CLEAR on front IR detection; TURN_GAP adds angular clearance.
        self.front_obstacle_threshold = 250
        self.front_clear_threshold = 155
        self.turn_speed = 90
        self.clear_count = 0
        self.clear_required = 7
        self.turn_direction = 1
        self.obstacle_turn_started = False

        # Extra turn after the front first becomes clear. This prevents the robot
        # from immediately turning back toward the goal, seeing the same obstacle,
        # and entering a SEEK/TURN_CLEAR oscillation.
        self.extra_clear_turn_degrees = 35.0
        self.extra_clear_start_theta = None

    def stop(self):
        """Command both wheels to stop; does not fetch a new hardware frame."""
        self.puck.epuck2.set_wheel_speeds(0, 0)

    def _front_value(self, prox):
        """Return the maximum of the four forward-facing proximity readings."""
        if not prox:
            return 0
        return max(prox[0], prox[7], prox[1], prox[6])

    def _right_value(self, prox):
        """Return the maximum reading from the right/front proximity group."""
        if not prox:
            return 0
        return max(prox[0], prox[1], prox[2], prox[3])

    def _left_value(self, prox):
        """Return the maximum reading from the left/front proximity group."""
        if not prox:
            return 0
        return max(prox[4], prox[5], prox[6], prox[7])

    def _angle_delta(self, start_theta, current_theta):
        """Return the signed heading change in degrees, wrapped to [-180, 180)."""
        return (current_theta - start_theta + 180.0) % 360.0 - 180.0

    def _choose_turn_direction(self, prox):
        """Choose the fixed left-pivot direction for team-consistent avoidance.

        Returning +1 makes the left wheel reverse and the right wheel advance.
        Avoiding sensor-dependent turn choices reduces the risk that teammates try
        to pass an obstacle on different sides.
        """
        return 1

    def step(self, frame):
        """Update odometry and execute one leader state-machine step.

        Args:
            frame: Fresh hardware frame containing cumulative motor_steps and prox.

        The controller selects SEEK, TURN_CLEAR, TURN_GAP, or STOP and issues wheel
        speed commands. It does not call epuck2.update() itself.
        """
        if self.finished:
            self.stop()
            return

        curr_x, curr_y, curr_theta = self.odom.update(*frame.motor_steps)
        self.last_position = (curr_x, curr_y, curr_theta)
        prox = list(frame.prox)
        front = self._front_value(prox)

        if self.state == "SEEK" and front > self.front_obstacle_threshold:
            self.state = "TURN_CLEAR"
            self.turn_direction = self._choose_turn_direction(prox)
            self.clear_count = 0
            self.extra_clear_start_theta = None
            self.obstacle_turn_started = True
            print(
                "Leader obstacle ahead. Pivoting in place until the front is clear "
                "front={} direction=LEFT fixed_team_rule.".format(
                    int(front),
                ),
                flush=True,
            )

        if self.state == "SEEK":
            self._step_seek(curr_x, curr_y, curr_theta)
        elif self.state == "TURN_CLEAR":
            self._step_turn_clear(prox, curr_theta)
        elif self.state == "TURN_GAP":
            self._step_turn_gap(prox, curr_theta)
        elif self.state == "STOP":
            self.finished = True
            self.stop()

    def _step_seek(self, curr_x, curr_y, curr_theta):
        """Steer toward the waypoint using the current odometry pose.

        The heading convention is 0 degrees along +Y, positive clockwise.
        The goal is reached within goal_tolerance_mm; otherwise the robot drives
        straight or applies asymmetric wheel speeds according to heading error.
        """
        dx = self.target_x - curr_x
        dy = self.target_y - curr_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= self.goal_tolerance_mm:
            print(
                "Goal reached. Final position=({:.1f}, {:.1f}, {:.1f}).".format(
                    curr_x,
                    curr_y,
                    curr_theta,
                ),
                flush=True,
            )
            self.state = "STOP"
            self.finished = True
            self.stop()
            return

        target_angle = math.degrees(math.atan2(dx, dy))
        angle_error = (target_angle - curr_theta + 180.0) % 360.0 - 180.0

        speed = int(self.speed)
        slow = int(self.speed * 0.45)

        if abs(angle_error) <= 13.0:
            left = speed
            right = speed
        elif angle_error > 0.0:
            left = speed
            right = slow
        else:
            left = slow
            right = speed

        self.puck.epuck2.set_wheel_speeds(left, right)

    def _pivot_in_place(self):
        """Command an in-place turn in the configured obstacle-avoidance direction."""
        left = -self.turn_direction * self.turn_speed
        right = self.turn_direction * self.turn_speed
        self.puck.epuck2.set_wheel_speeds(left, right)

    def _step_turn_clear(self, prox, curr_theta):
        """Pivot until the front IR readings remain clear for consecutive frames.

        After clear_required readings below front_clear_threshold, transition to
        TURN_GAP for additional angular clearance before resuming navigation.
        """
        front = self._front_value(prox)

        if front < self.front_clear_threshold:
            self.clear_count += 1
        else:
            self.clear_count = 0

        if self.clear_count >= self.clear_required:
            self.state = "TURN_GAP"
            self.extra_clear_start_theta = curr_theta
            self.clear_count = 0
            print(
                "Leader front is clear. Adding clearance gap before returning to SEEK "
                "front={} extra_turn={:.1f}deg.".format(
                    int(front),
                    self.extra_clear_turn_degrees,
                ),
                flush=True,
            )
            self._pivot_in_place()
            return

        self._pivot_in_place()

    def _step_turn_gap(self, prox, curr_theta):
        """Continue pivoting for an extra angle after clearing the front sensors.

        A renewed front obstruction returns the controller to TURN_CLEAR. Otherwise
        it resumes SEEK once the additional heading change reaches the configured
        extra_clear_turn_degrees.
        """
        front = self._front_value(prox)

        # If the front becomes blocked again during the extra gap, go back to
        # normal clearing and wait for stable clear readings again.
        if front > self.front_obstacle_threshold:
            self.state = "TURN_CLEAR"
            self.clear_count = 0
            self.extra_clear_start_theta = None
            print(
                "Leader front became blocked again during clearance gap. "
                "Continuing obstacle clear front={}.".format(int(front)),
                flush=True,
            )
            self._pivot_in_place()
            return

        if self.extra_clear_start_theta is None:
            self.extra_clear_start_theta = curr_theta

        turned = abs(self._angle_delta(self.extra_clear_start_theta, curr_theta))
        if turned >= self.extra_clear_turn_degrees:
            print(
                "Clearance gap complete. Returning to SEEK turned={:.1f}deg front={}.".format(
                    turned,
                    int(front),
                ),
                flush=True,
            )
            self.state = "SEEK"
            self.extra_clear_start_theta = None
            self.clear_count = 0
            return

        self._pivot_in_place()
