"""R&B formation follower with local obstacle reactions and wheel-speed control.

The follower captures an initial relative bearing/range target, then steers
using filtered bearing error and proportional range correction. Forward IR
obstacles trigger a fixed left pivot, while close side readings add gentle
steering without starting a team-wide obstacle hold. The controller receives
fresh R&B snapshots and hardware frames from the team mission; it does not
run its own communication or sensor loop.
"""


import math

from odometry import Odometry

from range_and_bearing_filter import wrap_degrees


class RangeBearingFollower:
    """Control a PiPuck follower's relative bearing and separation.

    Args:
        puck: Active PiPuck hardware interface.

    R&B range values are treated as sensor units, not automatically millimeters.
    Normal wheel commands are slew-limited; emergency pivot commands are not.
    """

    def __init__(self, puck):
        """Initialize formation capture, steering/range gains, and obstacle settings.

        The distance controller currently uses a proportional term only; its
        integral and derivative fields are retained for compatibility/diagnostics.
        """
        self.puck = puck

        self.odom = Odometry()


        self.target_bearing = None

        self.target_range = None

        self.target_peak = None


        self.capture_count = 0

        self.capture_required = 5


        self.bearing_deadband = 9.0

        self.rotate_only_error = 120.0

        self.filtered_error = 0.0

        self.error_alpha = 0.14

        self.kp_steer = 1.00

        self.maximum_turn = 36.0


        self.cruise_speed = 195.0

        self.minimum_base_speed = 150.0

        self.maximum_base_speed = 230.0


        self.range_deadband_fraction = 0.05

        self.range_deadband_minimum = 15.0

        self.range_gain = 0.28

        self.maximum_close_slowdown = 65.0

        self.maximum_far_speedup = 55.0


        self.kp_range = self.range_gain

        self.ki_range = 0.0

        self.kd_range = 0.0

        self.range_integral_limit = 0.0

        self.range_derivative_limit = 0.0

        self.range_pid_integral = 0.0

        self.range_pid_previous_error = None

        self.range_pid_previous_time = None


        self.maximum_wheel_speed = 230.0

        self.left_speed = 0.0

        self.right_speed = 0.0

        self.max_change_per_step = 28.0


        self.local_state = "FOLLOW"

        self.front_obstacle_threshold = 250

        self.front_clear_threshold = 155

        self.avoid_turn_speed = 90.0

        self.avoid_direction = 1

        self.clear_count = 0

        self.clear_required = 7

        self.local_gap_required_steps = 14

        self.local_gap_remaining_steps = 0

        # Bound the turn if the front sensors never clear.
        self.local_avoid_started = None
        self.local_obstacle_timeout_s = 30.0
        self.local_robot_yield_ratio = 0.90

        # Gentle side clearance is NOT an obstacle-bypass state: keep normal
        # R&B following and do not request a team-wide hold for side readings.
        # e-puck IR layout: [2] right side, [5] left side; [1]/[6] diagonals.
        self.side_steer_start_threshold = float(self.front_obstacle_threshold)
        self.side_steer_full_threshold = 500.0
        self.side_steer_max_turn = 12.0


    @property

    def formation_captured(self):
        """Report whether enough consistent R&B snapshots established a target."""
        return (

            self.target_bearing is not None

            and self.target_range is not None

            and self.target_peak is not None

            and self.capture_count >= self.capture_required

        )


    def reset_capture(self):
        """Discard the captured formation and reset local control history."""
        self.target_bearing = None

        self.target_range = None

        self.target_peak = None

        self.capture_count = 0

        self.filtered_error = 0.0

        self.left_speed = 0.0

        self.right_speed = 0.0

        self.local_state = "FOLLOW"

        self.local_gap_remaining_steps = 0

        self.range_pid_integral = 0.0

        self.range_pid_previous_error = None

        self.range_pid_previous_time = None


    def stop(self):
        """Reset stored wheel commands and request zero hardware wheel speeds."""
        self.left_speed = 0.0

        self.right_speed = 0.0

        self.puck.epuck2.set_wheel_speeds(0, 0)


    def hold(self, frame=None):
        """Pause local following and optionally integrate the latest encoder frame.

        Returns:
            Status dictionary indicating HOLD with zero wheel commands.
        """
        if frame is not None:

            self.odom.update(*frame.motor_steps)


        self.local_state = "HOLD"

        self.stop()

        return {

            "local_state": self.local_state,

            "front": 0.0,

            "left": 0,

            "right": 0,

        }


    def capture_step(self, snapshot):
        """Capture a stable relative bearing, range, and signal peak from R&B.

        Five consistent snapshots are required. A bearing change of more than
        10 degrees resets the streak; otherwise measurements are exponentially
        averaged. Wheel motion remains stopped during capture.

        Returns:
            True once the formation target has been captured; otherwise False.
        """
        self.local_state = "CAPTURE"

        self.stop()


        if snapshot is None:

            return False


        bearing = float(snapshot.bearing_degrees)

        range_value = float(snapshot.range_median)

        peak = float(snapshot.peak_median)


        if range_value <= 0.0 or peak <= 0.0:

            return False


        if self.target_bearing is None:

            self.target_bearing = bearing

            self.target_range = range_value

            self.target_peak = peak

            self.capture_count = 1

            return False


        bearing_change = abs(wrap_degrees(bearing - self.target_bearing))


        if bearing_change <= 10.0:

            self.target_bearing = 0.75 * self.target_bearing + 0.25 * bearing

            self.target_range = 0.75 * self.target_range + 0.25 * range_value

            self.target_peak = 0.75 * self.target_peak + 0.25 * peak

            self.capture_count += 1

        else:

            self.target_bearing = bearing

            self.target_range = range_value

            self.target_peak = peak

            self.capture_count = 1


        return self.formation_captured


    def _front_value(self, frame):
        """Read the maximum front-cone IR value, excluding the dedicated side sensors."""
        if frame is None or not getattr(frame, "prox", None):
            return 0
        prox = list(frame.prox)
        if len(prox) < 8:
            return 0
        # Side sensors [2] and [5] are NOT in the front cone.
        return max(prox[0], prox[1], prox[6], prox[7])


    def _side_steering_correction(self, frame):
        """Compute gentle avoidance steering from the two side IR sensors.

        Positive correction steers left, away from an object on the right. This does
        not identify whether the detected object is a wall or a teammate. Front
        obstacle handling takes priority, and this routine does not change state.

        Returns:
            Tuple (turn_correction, left_side_reading, right_side_reading).
        """
        if frame is None or getattr(frame, 'prox', None) is None:
            return 0.0, 0.0, 0.0
        prox = frame.prox
        if len(prox) < 8:
            return 0.0, 0.0, 0.0

        # Side steering uses only dedicated side sensors; forward diagonals
        # [1] and [6] are handled by the front-obstacle check instead.
        right = max(0.0, float(prox[2]))
        left = max(0.0, float(prox[5]))

        start = self.side_steer_start_threshold
        span = max(1.0, self.side_steer_full_threshold - start)
        right_strength = max(0.0, min(1.0, (right - start) / span))
        left_strength = max(0.0, min(1.0, (left - start) / span))
        correction = self.side_steer_max_turn * (right_strength - left_strength)
        return correction, left, right


    def _choose_avoid_direction(self, frame):
            """Choose the fixed +1 (left-pivot) obstacle-avoidance direction."""

            self.avoid_direction = 1

            return self.avoid_direction


    def _local_obstacle_status(self, front, range_state):
        """Build a diagnostic status dictionary while local obstacle control owns the wheels."""
        return {

            "local_state": self.local_state,

            "front": float(front),

            "bearing": None,

            "target_bearing": None,

            "bearing_error": None,

            "range": None,

            "target_range": None,

            "range_error": None,

            "range_deadband": None,

            "range_state": str(range_state),

            "peak": None,

            "peak_ratio": None,

            "speed_scale": 0.0,

            "left": int(self.left_speed),

            "right": int(self.right_speed),

        }


    def _pivot_in_place(self):
        """Turn in place immediately, bypassing normal wheel-speed slew limiting.

        This is used for local front-obstacle avoidance so the robot does not
        continue translating forward while its wheel speeds ramp toward a pivot.
        """
        target_left = -self.avoid_direction * self.avoid_turn_speed

        target_right = self.avoid_direction * self.avoid_turn_speed


        # Bypass slew limiting so front-obstacle pivots begin immediately.

        self.left_speed = target_left

        self.right_speed = target_right


        self.puck.epuck2.set_wheel_speeds(

            int(self.left_speed),

            int(self.right_speed),

        )

    def _local_avoid_step(self, frame, snapshot=None):
        """Manage local front-obstacle yielding, clearing, and timeout states.

        A nearby leader identified by the R&B snapshot can cause a stationary yield.
        Other front obstructions trigger a left pivot. Seven consecutive front-clear
        readings return control to the normal R&B follower in the same call, with no
        extra gap maneuver. A turn exceeding 30 seconds enters LOCAL_BLOCKED.

        Returns:
            An obstacle-status dictionary while avoiding, or None when normal
            R&B following may resume.
        """
        import time
        front = self._front_value(frame)
        if frame is None or getattr(frame, 'prox', None) is None or len(frame.prox) < 8:
            self.stop()
            return self._local_obstacle_status(front, 'NO_VALID_SENSOR_FRAME')

        # A nearby, identified leader can be the IR object. Keep the previous
        # conservative yield heuristic, but do not interrupt a committed turn.
        close_leader_in_front = False
        if snapshot is not None and self.target_range is not None:
            try:
                close_leader_in_front = (
                    abs(float(snapshot.bearing_degrees)) <= 27.0
                    and 0.0 < float(snapshot.range_median)
                    < self.local_robot_yield_ratio * float(self.target_range)
                )
            except (TypeError, ValueError):
                pass

        if (self.local_state in ('FOLLOW', 'LOCAL_YIELD')
                and front >= self.front_obstacle_threshold
                and close_leader_in_front):
            self.local_state = 'LOCAL_YIELD'
            self.stop()
            return self._local_obstacle_status(front, 'YIELD_NEAR_LEADER')
        if self.local_state == 'LOCAL_YIELD':
            self.local_state = 'FOLLOW'
            if front < self.front_obstacle_threshold:
                return None

        if self.local_state not in ('LOCAL_TURN_CLEAR', 'LOCAL_BLOCKED'):
            self.local_state = 'LOCAL_TURN_CLEAR'
            self.avoid_direction = 1  # Team-wide left-turn convention.
            self.clear_count = 0
            self.local_avoid_started = time.monotonic()
            print('FOLLOWER OBSTACLE: turning until front is clear; team holds.', flush=True)

        if self.local_state == 'LOCAL_BLOCKED':
            self.stop()
            return self._local_obstacle_status(front, 'BLOCKED_OPERATOR_REQUIRED')

        if (self.local_avoid_started is not None
                and time.monotonic() - self.local_avoid_started > self.local_obstacle_timeout_s):
            self.local_state = 'LOCAL_BLOCKED'
            self.stop()
            print('FOLLOWER OBSTACLE: front never cleared; stopping safely.', flush=True)
            return self._local_obstacle_status(front, 'TURN_TIMEOUT')

        self.clear_count = self.clear_count + 1 if front < self.front_clear_threshold else 0
        if self.clear_count >= self.clear_required:
            # Crucial: hand control back to the R&B PID NOW, not after an extra
            # angle/side-distance bypass. Reset pivot command before PID slew.
            self.local_state = 'FOLLOW'
            self.clear_count = 0
            self.local_avoid_started = None
            self.stop()
            print('FOLLOWER OBSTACLE: front clear; resuming R&B follow.', flush=True)
            return None

        self._pivot_in_place()
        return self._local_obstacle_status(front, 'TURN_UNTIL_CLEAR')


    def _steering_error(self, measured_bearing):
        """Calculate wrapped bearing error, apply a deadband, and low-pass filter it.

        Returns:
            Tuple (raw_error_degrees, filtered_control_error_degrees).
        """
        raw_error = wrap_degrees(float(measured_bearing) - float(self.target_bearing))


        if abs(raw_error) <= self.bearing_deadband:

            deadband_error = 0.0

        elif raw_error > 0.0:

            deadband_error = raw_error - self.bearing_deadband

        else:

            deadband_error = raw_error + self.bearing_deadband


        self.filtered_error = (

            (1.0 - self.error_alpha) * self.filtered_error

            + self.error_alpha * deadband_error

        )


        return raw_error, self.filtered_error


    def _reset_range_pid(self):
        """Clear the stored range-controller history and diagnostic state."""
        self.range_pid_integral = 0.0

        self.range_pid_previous_error = None

        self.range_pid_previous_time = None


    def _range_speed(self, measured_range, now_seconds=None):
        """Compute base wheel speed from separation error using proportional control.

        Applies a distance deadband, asymmetric close/far correction limits, and
        minimum/maximum base-speed bounds. Despite legacy PID-related field names,
        integral and derivative terms do not contribute to the output.

        Returns:
            An eight-element tuple containing base speed, raw error, deadband,
            controller state, effective error, zero integral, zero derivative,
            and proportional speed correction.
        """
        range_error = float(measured_range) - float(self.target_range)


        deadband = max(

            self.range_deadband_minimum,

            abs(self.target_range) * self.range_deadband_fraction,

        )


        if abs(range_error) <= deadband:

            effective_error = 0.0

        elif range_error > 0.0:

            effective_error = range_error - deadband

        else:

            effective_error = range_error + deadband


        correction = self.range_gain * effective_error

        correction = max(

            -self.maximum_close_slowdown,

            min(self.maximum_far_speedup, correction),

        )


        base_speed = self.cruise_speed + correction

        base_speed = max(

            self.minimum_base_speed,

            min(self.maximum_base_speed, base_speed),

        )


        if effective_error == 0.0:

            range_state = "PID_ON_DISTANCE"

        elif effective_error > 0.0:

            range_state = "PID_FAR_PROPORTIONAL"

        else:

            range_state = "PID_CLOSE_PROPORTIONAL"


        self.range_pid_integral = 0.0

        self.range_pid_previous_error = effective_error

        self.range_pid_previous_time = now_seconds


        return (

            base_speed,

            range_error,

            deadband,

            range_state,

            effective_error,

            0.0,

            0.0,

            correction,

        )


    def _slew(self, current, target):
        """Limit a single wheel command change to max_change_per_step."""
        difference = target - current


        if difference > self.max_change_per_step:

            difference = self.max_change_per_step

        elif difference < -self.max_change_per_step:

            difference = -self.max_change_per_step


        return current + difference


    def follow_step(self, snapshot, frame=None, speed_scale=1.0, wifi_state=None, my_id=None, leader_id=None):
        """Perform one normal follower update from R&B and optional local IR sensors.

        Args:
            snapshot: Current R&B filter snapshot with bearing, range, and peak data.
            frame: Optional fresh hardware frame for odometry and proximity sensing.
            speed_scale: Multiplicative forward-speed factor, clamped to [0, 1].
            wifi_state: Accepted for compatibility; not used by this method.
            my_id: Accepted for compatibility; not used by this method.
            leader_id: Accepted for compatibility; not used by this method.

        Returns:
            Diagnostic status dictionary, or None if formation capture is incomplete.

        Local front-obstacle handling has priority over bearing/range control. Side
        IR readings can alter steering without activating a separate avoidance state.
        """
        if frame is not None:

            self.odom.update(*frame.motor_steps)


        if not self.formation_captured:

            self.stop()

            return None


        front = self._front_value(frame)


        if self.local_state in ("LOCAL_TURN_CLEAR", "LOCAL_BLOCKED", "LOCAL_YIELD"):

            avoid_status = self._local_avoid_step(frame, snapshot=snapshot)

            if avoid_status is not None:

                return avoid_status


        if front > self.front_obstacle_threshold:

            avoid_status = self._local_avoid_step(frame, snapshot=snapshot)

            if avoid_status is not None:

                return avoid_status


        self.local_state = "FOLLOW"

        self.clear_count = 0

        self.local_gap_remaining_steps = 0


        speed_scale = max(0.0, min(1.0, float(speed_scale)))


        raw_error, control_error = self._steering_error(

            snapshot.bearing_degrees

        )


        (

            base_speed,

            range_error,

            range_deadband,

            range_state,

            range_pid_error,

            range_pid_integral,

            range_pid_derivative,

            range_pid_output,

        ) = self._range_speed(snapshot.range_median)


        if base_speed > 0.0:

            base_speed *= speed_scale


        if abs(raw_error) >= self.rotate_only_error and base_speed > 0.0:

            base_speed = 0.0


        # R&B remains the main steering controller. If an object is close to
        # one side, give a modest correction AWAY from that side. Opposing side
        # readings cancel; front obstacles have already been handled above.
        rab_turn = max(
            -self.maximum_turn,
            min(self.maximum_turn, self.kp_steer * control_error),
        )
        side_correction, side_left, side_right = self._side_steering_correction(frame)
        if side_correction > 0.0:
            # Positive turn = left wheel slower, right wheel faster = LEFT.
            turn = max(rab_turn, side_correction)
        elif side_correction < 0.0:
            # Negative turn = RIGHT.
            turn = min(rab_turn, side_correction)
        else:
            turn = rab_turn


        target_left = max(

            -self.maximum_wheel_speed,

            min(self.maximum_wheel_speed, base_speed - turn),

        )

        target_right = max(

            -self.maximum_wheel_speed,

            min(self.maximum_wheel_speed, base_speed + turn),

        )


        self.left_speed = self._slew(self.left_speed, target_left)

        self.right_speed = self._slew(self.right_speed, target_right)

        # Independent wheel slew limits can briefly make BOTH wheels equal
        # during acceleration, masking the side correction. Ensure a small
        # differential immediately when the side sensor is active; otherwise
        # leave the original wheel-slew behavior completely unchanged.
        if side_correction != 0.0:
            desired_diff = 2.0 * side_correction  # right wheel minus left wheel
            actual_diff = self.right_speed - self.left_speed
            if (side_correction > 0 and actual_diff < desired_diff) or (
                    side_correction < 0 and actual_diff > desired_diff):
                average = 0.5 * (self.left_speed + self.right_speed)
                self.left_speed = max(-self.maximum_wheel_speed,
                                      min(self.maximum_wheel_speed,
                                          average - side_correction))
                self.right_speed = max(-self.maximum_wheel_speed,
                                       min(self.maximum_wheel_speed,
                                           average + side_correction))


        self.puck.epuck2.set_wheel_speeds(

            int(self.left_speed),

            int(self.right_speed),

        )


        peak_ratio = 1.0

        if self.target_peak > 0.0:

            peak_ratio = (

                float(snapshot.peak_median)

                / float(self.target_peak)

            )


        return {

            "local_state": self.local_state,

            "front": float(front),

            "side_left": float(side_left),
            "side_right": float(side_right),
            "side_steer": float(side_correction),

            "bearing": float(snapshot.bearing_degrees),

            "target_bearing": float(self.target_bearing),

            "bearing_error": float(raw_error),

            "range": float(snapshot.range_median),

            "target_range": float(self.target_range),

            "range_error": float(range_error),

            "range_deadband": float(range_deadband),

            "range_state": str(range_state),

            "range_pid_error": float(range_pid_error),

            "range_pid_integral": float(range_pid_integral),

            "range_pid_derivative": float(range_pid_derivative),

            "range_pid_output": float(range_pid_output),

            "base_speed": float(base_speed),

            "peak": float(snapshot.peak_median),

            "peak_ratio": float(peak_ratio),

            "speed_scale": float(speed_scale),

            "left": int(self.left_speed),

            "right": int(self.right_speed),

        }


    def recovery_step(self, frame, elapsed, last_bearing_error):
        """Hold the follower stationary and report elapsed recovery information.

        This routine does not perform Wi-Fi or blind navigation; those behaviors
        are delegated to the separate recovery controller.
        """
        self.local_state = "HOLD"

        self.stop()

        return {

            "local_state": self.local_state,

            "elapsed": float(elapsed),

            "front": float(self._front_value(frame)),

            "left": 0,

            "right": 0,

        }

    def calculate_recovery_target(self, leader_wifi_x, leader_wifi_y, leader_wifi_theta):
        """Compute a candidate formation-offset target from the leader's reported pose.

        The calculation assumes target_range is a distance in millimeters, that
        the leader heading is in radians, and that poses use compatible frames.
        Those assumptions must be validated by the caller; R&B sensor range and
        separate odometry origins do not themselves establish them.
        """

        target_bearing_rad = math.radians(self.target_bearing)

        distance_mm = self.target_range

        global_angle = leader_wifi_theta + target_bearing_rad

        recovery_x = leader_wifi_x - (distance_mm * math.sin(global_angle))

        recovery_y = leader_wifi_y - (distance_mm * math.cos(global_angle))

        return recovery_x, recovery_y