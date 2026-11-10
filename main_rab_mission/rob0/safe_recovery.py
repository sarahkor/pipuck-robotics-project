"""Follower recovery controllers for periods without reliable R&B tracking.

TimedBlindRecovery moves cautiously forward with front-obstacle checks.
WiFiDisplacementRecovery additionally follows a fresh Wi-Fi displacement
waypoint, while retaining a time-limited blind fallback. The other two classes,
FormationTarget and SafeRecoveryNavigator, implement an alternative formation
transform and boundary-following recovery strategy.

Waypoint navigation uses +x to the right and +y forward, with headings in
DEGREES clockwise from +y. Odometry.theta is in radians; callers must convert
it before passing a heading to these controllers. Recovery is reactive and
bounded, not a guarantee of collision-free or globally complete navigation.
"""

import math
import time


def wrap_degrees(angle):
    """Normalize a heading in degrees to the half-open range [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def rotate_clockwise(x, y, angle_degrees):
    """Rotate a 2D displacement clockwise by ``angle_degrees``.
    
        Args:
            x: Rightward component in the original frame.
            y: Forward component in the original frame.
            angle_degrees: Clockwise rotation in degrees.
    
        Returns:
            The rotated ``(x, y)`` components.
        """
    angle = math.radians(angle_degrees)
    c, s = math.cos(angle), math.sin(angle)
    return x * c + y * s, -x * s + y * c


class FormationTarget:
    """Map the leading robot's local odometry to the follower's initial frame.

    Assumptions:
      * At capture, leader odometry is (0,0,0) and follower is stationary.
      * leader_heading_offset_deg is the leader's *initial* physical heading
        minus the follower's initial physical heading (clockwise-positive).
      * R&B bearing is degrees COUNTERCLOCKWISE (positive LEFT), as indicated
        by RangeBearingFollower steering: positive bearing error -> turn left.
      * separation_mm is calibrated robot-center-to-robot-center distance.
      * Later leader poses come from the same continuously running odometry.
    """

    def __init__(self, follower_pose, bearing_degrees, separation_mm,
                 leader_heading_offset_deg=0.0):
        """Capture a fixed physical formation offset and frame rotation.
        
                Args:
                    follower_pose: Initial follower ``(x, y, heading_degrees)``.
                    bearing_degrees: Initial R&B bearing toward the leader, positive left.
                    separation_mm: Calibrated center-to-center distance in millimeters.
                    leader_heading_offset_deg: Initial leader heading minus follower
                        heading, clockwise-positive in degrees.
        
                Raises:
                    ValueError: If the supplied separation is nonpositive or nonfinite.
                """
        distance = float(separation_mm)
        if not math.isfinite(distance) or distance <= 0.0:
            raise ValueError('Initial center separation must be positive millimeters')
        self.follower_start = tuple(map(float, follower_pose))
        fx, fy, ftheta = self.follower_start
        angle = math.radians(ftheta - float(bearing_degrees))
        self.offset_x = distance * math.sin(angle)  # follower -> leader
        self.offset_y = distance * math.cos(angle)
        self.leader_start_x = fx + self.offset_x
        self.leader_start_y = fy + self.offset_y
        self.heading_offset = float(leader_heading_offset_deg)

    def target(self, leader_pose):
        """Estimate the follower waypoint for a leader odometry pose.
        
                Args:
                    leader_pose: Leader displacement and heading ``(x, y, theta)``;
                        coordinates are in the leader's own initial odometry frame and
                        heading is clockwise-positive degrees.
        
                Returns:
                    Target ``(x, y)`` in the follower's initial odometry frame.
        
                Raises:
                    ValueError: If the supplied leader pose contains nonfinite values.
                """
        lx, ly, ltheta = tuple(map(float, leader_pose))
        if not all(math.isfinite(v) for v in (lx, ly, ltheta)):
            raise ValueError('Invalid leader odometry state')
        # Rotate the leader's displacement from its OWN frame into the
        # follower's initial frame. Then rotate the formation offset as
        # the leader changes heading from its initial heading 0.
        disp_x, disp_y = rotate_clockwise(lx, ly, self.heading_offset)
        leader_x = self.leader_start_x + disp_x
        leader_y = self.leader_start_y + disp_y
        off_x, off_y = rotate_clockwise(self.offset_x, self.offset_y,
                                       ltheta)
        return leader_x - off_x, leader_y - off_y


class SafeRecoveryNavigator:
    """Slow reactive boundary following with fail-closed time limits.

    All calls to step() must supply a NEW proximity/encoder sensor frame and
    the ALREADY-UPDATED follower pose. This class never integrates odometry.
    """
    def __init__(self, puck):
        """Initialize the boundary-following recovery state and safety limits.
        
                Args:
                    puck: PiPuck instance used to command wheel speeds.
                """
        self.puck = puck
        self.state = 'SEEK'
        self.finished = False
        self.failed = False
        self.reason = None
        self.left = 0
        self.right = 0

        self.speed = 105
        self.turn_speed = 65
        self.goal_tolerance_mm = 45.0
        self.front_obstacle_threshold = 190  # Tune using the actual sensors!
        self.front_clear_threshold = 115
        self.side_too_close_threshold = 210
        self.side_present_threshold = 75
        self.side_rejoin_threshold = 95
        self.minimum_pivot_degrees = 48.0
        self.minimum_bypass_distance_mm = 125.0
        self.minimum_goal_progress_mm = 55.0
        self.max_wall_seconds = 35.0
        self.max_recovery_seconds = 60.0
        self.start_time = time.monotonic()
        self.hit_distance = None
        self.avoid_start = None
        self.avoid_pose = None
        self.pivot_theta = None
        self.wall_progress_mm = 0.0
        self.last_wall_pose = None
        self.clear_frames = 0

    def _wheels(self, left, right):
        """Send wheel-speed commands and remember the last commanded values."""
        self.left, self.right = int(left), int(right)
        self.puck.epuck2.set_wheel_speeds(self.left, self.right)

    def stop(self):
        """Command zero wheel speeds without changing the recovery state."""
        self._wheels(0, 0)

    def _fail(self, why):
        """Mark recovery as failed, stop both wheels, and report the reason."""
        self.failed = True
        self.reason = why
        self.state = 'FAILED'
        self.stop()
        print('RECOVERY SAFETY STOP: {}'.format(why), flush=True)

    def _pivot_left(self):
        """Command an in-place left pivot using opposing wheel speeds."""
        self._wheels(-self.turn_speed, self.turn_speed)

    def _start_avoid(self, pose, goal_distance):
        """Enter obstacle avoidance and record the initial goal distance and pose.
        
                Args:
                    pose: Current follower ``(x, y, heading_degrees)``.
                    goal_distance: Distance to the current waypoint, in millimeters.
                """
        self.state = 'PIVOT_LEFT'
        self.pivot_theta = float(pose[2])
        self.avoid_start = time.monotonic()
        self.avoid_pose = (float(pose[0]), float(pose[1]))
        self.last_wall_pose = None
        self.wall_progress_mm = 0.0
        self.clear_frames = 0
        self.hit_distance = float(goal_distance)
        print('RECOVERY: obstacle encountered, pivoting left', flush=True)
        self._pivot_left()

    @staticmethod
    def _front(prox):
        """Return the strongest reading among the four front proximity sensors."""
        return max(prox[0], prox[1], prox[6], prox[7])

    def step(self, frame, follower_pose, target_x, target_y):
        """Advance one boundary-following navigation step toward a waypoint.
        
                Uses SEEK, PIVOT_LEFT, and FOLLOW_BOUNDARY states. A front obstacle
                overrides goal steering; reaching the target or violating a safety
                limit stops the wheels. Call with one fresh sensor frame and a pose
                whose odometry has already been updated for that frame.
        
                Args:
                    frame: Current sensor frame containing at least eight ``prox`` values.
                    follower_pose: ``(x_mm, y_mm, heading_degrees)``.
                    target_x: Desired rightward coordinate in millimeters.
                    target_y: Desired forward coordinate in millimeters.
        
                Returns:
                    Current state name, such as ``SEEK``, ``AT_TARGET`` or ``FAILED``.
                """
        if self.finished or self.failed:
            self.stop()
            return self.state
        if frame is None or getattr(frame, 'prox', None) is None or len(frame.prox) < 8:
            self._fail('missing proximity frame')
            return self.state
        now = time.monotonic()
        if now - self.start_time > self.max_recovery_seconds:
            self._fail('recovery time limit exceeded')
            return self.state
        x, y, theta = tuple(map(float, follower_pose))
        target_x, target_y = float(target_x), float(target_y)
        if not all(math.isfinite(v) for v in (x, y, theta, target_x, target_y)):
            self._fail('non-finite pose or target')
            return self.state
        prox = list(frame.prox)
        front = self._front(prox)
        right_side = max(prox[1], prox[2])
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)
        desired_heading = math.degrees(math.atan2(dx, dy))
        error = wrap_degrees(desired_heading - theta)

        if distance <= self.goal_tolerance_mm:
            self.finished = True
            self.state = 'AT_TARGET'
            self.stop()  # Never keep blindly driving when R&B is still missing.
            return self.state

        if self.state == 'SEEK':
            # A turn toward the goal must never override a front obstacle.
            if front >= self.front_obstacle_threshold:
                self._start_avoid((x, y, theta), distance)
            elif abs(error) > 25:
                # Rotate without forward translation when misaligned.
                if error > 0:
                    self._wheels(self.turn_speed, -self.turn_speed)
                else:
                    self._pivot_left()
            else:
                # Positive heading error means RIGHT turn: left wheel faster.
                steer = max(-35, min(35, 1.1 * error))
                self._wheels(max(40, self.speed + steer),
                             max(40, self.speed - steer))
            return self.state

        if self.state == 'PIVOT_LEFT':
            if front < self.front_clear_threshold:
                self.clear_frames += 1
            else:
                self.clear_frames = 0
            turned = abs(wrap_degrees(theta - self.pivot_theta))
            if self.clear_frames >= 4 and turned >= self.minimum_pivot_degrees:
                self.state = 'FOLLOW_BOUNDARY'
                self.last_wall_pose = (x, y)
                self.wall_progress_mm = 0.0
                print('RECOVERY: front clear; following right-side boundary', flush=True)
            self._pivot_left()
            return self.state

        if self.state == 'FOLLOW_BOUNDARY':
            if now - self.avoid_start > self.max_wall_seconds:
                self._fail('cannot safely bypass obstacle in time')
                return self.state
            if self.last_wall_pose is not None:
                self.wall_progress_mm += math.hypot(x - self.last_wall_pose[0],
                                                    y - self.last_wall_pose[1])
            self.last_wall_pose = (x, y)
            # Never translate when anything is close in the front cone.
            if front >= self.front_obstacle_threshold:
                self._pivot_left()
                return self.state
            # Rejoin only after physically travelling around the obstacle AND
            # improving goal distance AND obtaining a relatively clear side.
            if (self.wall_progress_mm >= self.minimum_bypass_distance_mm
                    and distance <= self.hit_distance - self.minimum_goal_progress_mm
                    and abs(error) < 48
                    and front < self.front_clear_threshold
                    and right_side < self.side_rejoin_threshold):
                self.state = 'SEEK'
                print('RECOVERY: clear path toward target; rejoining', flush=True)
                self._wheels(0, 0)
                return self.state
            if right_side > self.side_too_close_threshold:
                # Obstacle near the RIGHT: turn gently LEFT.
                self._wheels(50, 105)
            elif right_side < self.side_present_threshold:
                # No wall on RIGHT: curve gently RIGHT to find/follow it.
                self._wheels(110, 65)
            else:
                self._wheels(90, 90)
            return self.state

        self._fail('unexpected recovery state: {}'.format(self.state))
        return self.state


class TimedBlindRecovery:
    """At most 10 seconds of forward motion, with IR-triggered pivot-to-clear.

    This is only for missing R&B: it cannot steer toward a leader it cannot
    observe. The mission immediately returns to normal FOLLOW on R&B recovery.
    """
    def __init__(self, puck, duration_seconds=10.0, clock=None):
        """Begin a time-limited blind recovery period.
        
                Args:
                    puck: PiPuck instance used to command wheel speeds.
                    duration_seconds: Maximum elapsed time before blind recovery stops.
                    clock: Optional monotonic-style clock, useful for deterministic tests.
                """
        self.puck = puck
        self.clock = time.monotonic if clock is None else clock
        self.started = self.clock()
        self.duration_seconds = float(duration_seconds)
        self.state = 'FORWARD'
        self.done = False
        self.reason = None
        self.left = self.right = 0
        self.forward_speed = 150
        self.turn_speed = 90
        self.front_obstacle_threshold = 250
        self.front_clear_threshold = 155
        self.clear_required = 7
        self.clear_frames = 0
        self.obstacle_unresolved = False

    @property
    def obstacle_active(self):
        """Whether an obstacle is still unresolved or the robot is turning clear."""
        return self.obstacle_unresolved or self.state == 'TURN_CLEAR'

    def _wheels(self, left, right):
        """Command both wheels and store their latest commanded speeds."""
        self.left, self.right = int(left), int(right)
        self.puck.epuck2.set_wheel_speeds(self.left, self.right)

    def stop(self):
        """Stop both wheels without marking the recovery as finished."""
        self._wheels(0, 0)

    def _finish(self, reason):
        # If still obstructed, keep reporting obstacle hold while awaiting R&B.
        """Finish recovery with ``reason`` and preserve any unresolved obstacle flag."""
        self.obstacle_unresolved = self.obstacle_active
        self.done = True
        self.reason = reason
        self.state = 'EXPIRED' if reason == '10-second limit' else 'UNSAFE'
        self.stop()

    def step(self, frame, heading_degrees, xy=None):
        """Advance blind forward recovery or an IR-triggered clearance turn.
        
                Front proximity readings trigger TURN_CLEAR, which pivots left until
                the front stays below the clearing threshold for the required number
                of controller calls. The caller should supply distinct sensor frames.
                The heading and position arguments are accepted for interface
                compatibility but are not used by this implementation.
        
                Args:
                    frame: Sensor frame containing at least eight proximity readings.
                    heading_degrees: Current heading, currently unused.
                    xy: Optional position, currently unused.
        
                Returns:
                    Current state (``FORWARD``, ``TURN_CLEAR``, ``EXPIRED`` or ``UNSAFE``).
                """
        if self.done:
            self.stop()
            return self.state
        if self.clock() - self.started >= self.duration_seconds:
            self._finish('10-second limit')
            return self.state
        if frame is None or getattr(frame, 'prox', None) is None:
            self._finish('missing proximity frame')
            return self.state
        try:
            prox = list(frame.prox)
            if len(prox) < 8:
                raise ValueError('incomplete proximity frame')
            front = max(float(prox[i]) for i in (0, 1, 6, 7))
            if not math.isfinite(front):
                raise ValueError('nonfinite proximity value')
        except (TypeError, ValueError, IndexError):
            self._finish('invalid proximity frame')
            return self.state

        if self.state == 'FORWARD':
            if front >= self.front_obstacle_threshold:
                self.state = 'TURN_CLEAR'
                self.clear_frames = 0
                print('BLIND RECOVERY OBSTACLE: pivoting until front is clear.', flush=True)
                self._wheels(-self.turn_speed, self.turn_speed)
            else:
                self._wheels(self.forward_speed, self.forward_speed)
            return self.state

        if self.state == 'TURN_CLEAR':
            self.clear_frames = self.clear_frames + 1 if front < self.front_clear_threshold else 0
            if self.clear_frames >= self.clear_required:
                self.state = 'FORWARD'
                self.clear_frames = 0
                print('BLIND RECOVERY OBSTACLE: front clear; continuing forward.', flush=True)
                self._wheels(self.forward_speed, self.forward_speed)
                return self.state
            self._wheels(-self.turn_speed, self.turn_speed)
            return self.state

        self._finish('unexpected recovery state')
        return self.state


class WiFiDisplacementRecovery(TimedBlindRecovery):
    """R&B-loss recovery using a fresh Wi-Fi *odometry displacement* target.

    The leading robot's (x, y) is interpreted as the follower's desired (x, y)
    in its OWN odometry coordinates. This reproduces the leader's displacement
    while retaining the physical offset between robots at mission start.

    REQUIRED physical setup: both robot odometry origins are zeroed at the
    beginning of the mission and their initial headings/axes are aligned.
    A common numeric (0, 0, 0) without a common physical orientation is NOT
    sufficient. Wheel slip or any pose-frame-changing Wi-Fi fusion can invalidate
    the displacement correspondence; this is a best-effort fallback, never more
    authoritative than a fresh R&B bearing/range observation.

    Only call step() with NEW hardware frames; own odometry must already have
    integrated the frame exactly once. This class NEVER integrates odometry.
    It preserves the blind recovery front-IR thresholds / pivot-to-clear
    (250 detection, 155 clearing, 7 distinct frames, wheel pivot 90), and
    uses the same near-only side sensors [2] and [5] at threshold 250.
    """

    def __init__(self, puck, duration_seconds=10.0, clock=None):
        """Initialize Wi-Fi waypoint steering and its independent timeout.
        
                Args:
                    puck: PiPuck hardware interface.
                    duration_seconds: Blind travel limit, in seconds.
                    clock: Optional injected clock for timing and tests.
                """
        super().__init__(puck, duration_seconds=duration_seconds, clock=clock)
        self.wifi_speed = 105
        self.wifi_turn_limit = 35.0
        self.wifi_rotate_error_degrees = 32.0
        self.wifi_goal_tolerance_mm = 45.0
        self.side_threshold = 250.0
        self.side_full_threshold = 500.0
        self.side_max_turn = 12.0
        self.wifi_recovery_limit = 60.0
        self.using_wifi_target = False
        self.target = None

    @staticmethod
    def _wrap_degrees(value):
        """Normalize a heading difference to [-180, 180) degrees."""
        return (float(value) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _side_strength(value, start, full):
        """Scale a side proximity reading to a clamped [0, 1] correction weight."""
        return max(0.0, min(1.0, (float(value) - start) / (full - start)))

    def _wifi_seek(self, prox, pose, target):
        """Steer toward a Wi-Fi displacement waypoint with gentle side avoidance.
        
                Rotates in place when the heading error is large and stops within the
                target tolerance. This helper assumes front-obstacle handling has
                already occurred in ``step``.
        
                Args:
                    prox: Proximity sensor readings (indices 2 and 5 are used).
                    pose: Follower ``(x_mm, y_mm, heading_degrees)``.
                    target: Desired ``(x_mm, y_mm)`` in the follower's odometry frame.
                """
        x, y, theta = pose
        tx, ty = target
        dx, dy = tx - x, ty - y
        distance = math.hypot(dx, dy)
        if distance <= self.wifi_goal_tolerance_mm:
            self.state = 'WIFI_AT_TARGET'
            self.stop()
            return
        self.state = 'WIFI_SEEK'
        desired = math.degrees(math.atan2(dx, dy))
        error = self._wrap_degrees(desired - theta)
        if abs(error) > self.wifi_rotate_error_degrees:
            # Heading is clockwise-positive: positive error means turn right.
            if error > 0:
                self._wheels(self.turn_speed, -self.turn_speed)
            else:
                self._wheels(-self.turn_speed, self.turn_speed)
            return
        steering = max(-self.wifi_turn_limit,
                       min(self.wifi_turn_limit, 1.0 * error))
        # Sensor [2] is right, sensor [5] is left. Strong right-side IR
        # steers LEFT by reducing the left wheel relative to the right.
        side_right = self._side_strength(prox[2], self.side_threshold,
                                         self.side_full_threshold)
        side_left = self._side_strength(prox[5], self.side_threshold,
                                        self.side_full_threshold)
        steering += self.side_max_turn * (side_left - side_right)
        self._wheels(max(30, self.wifi_speed + steering),
                     max(30, self.wifi_speed - steering))

    def step(self, frame, heading_degrees, xy=None, wifi_target=None):
        """Choose Wi-Fi waypoint tracking or a bounded blind-forward fallback.
        
                A present ``wifi_target`` takes priority over blind driving, while
                front obstacle detection takes priority over either movement mode.
                Fresh R&B takeover is managed by ``team_mission_rab.py``; this method
                does not read R&B directly. Call only with a new sensor frame after
                integrating its encoder readings exactly once into odometry.
        
                Args:
                    frame: Current hardware frame with at least eight ``prox`` readings.
                    heading_degrees: Follower heading in clockwise-positive degrees.
                    xy: Follower's own ``(x_mm, y_mm)`` odometry displacement.
                    wifi_target: Fresh front-robot displacement ``(x_mm, y_mm)`` in
                        aligned axes, or ``None`` if unavailable.
        
                Returns:
                    Current recovery state, e.g. ``WIFI_SEEK``, ``FORWARD``,
                    ``TURN_CLEAR``, ``WIFI_AT_TARGET``, ``EXPIRED`` or ``UNSAFE``.
                """
        if self.done:
            self.stop()
            return self.state
        if frame is None or getattr(frame, 'prox', None) is None:
            self._finish('missing proximity frame')
            return self.state
        try:
            prox = list(frame.prox)
            if len(prox) < 8:
                raise ValueError('incomplete proximity frame')
            front = max(float(prox[i]) for i in (0, 1, 6, 7))
            heading_degrees = float(heading_degrees)
            if xy is None:
                raise ValueError('missing odometry pose')
            x, y = float(xy[0]), float(xy[1])
            target = None if wifi_target is None else (
                float(wifi_target[0]), float(wifi_target[1]))
            values = (front, x, y, heading_degrees) + (() if target is None else target)
            if not all(math.isfinite(value) for value in values):
                raise ValueError('nonfinite sensor/odometry')
        except (TypeError, ValueError, IndexError, OverflowError):
            self._finish('invalid proximity/odometry frame')
            return self.state

        self.using_wifi_target = target is not None
        self.target = target
        now = self.clock()
        # Do not drive blindly past the original 10-second cap. Wi-Fi
        # navigation has its own bounded lifetime to avoid endless drifting.
        if not self.using_wifi_target and now - self.started >= self.duration_seconds:
            self._finish('10-second limit without Wi-Fi or R&B')
            return self.state
        if now - self.started >= self.wifi_recovery_limit:
            self._finish('Wi-Fi recovery safety limit')
            return self.state

        # A front obstacle overrides BOTH Wi-Fi waypoint steering and blind
        # forward travel. Preserve the turn state across Wi-Fi dropouts.
        if self.state == 'TURN_CLEAR':
            self.clear_frames = self.clear_frames + 1 if front < self.front_clear_threshold else 0
            if self.clear_frames >= self.clear_required:
                self.clear_frames = 0
                self.obstacle_unresolved = False
                if target is None:
                    self.state = 'FORWARD'
                    self._wheels(self.forward_speed, self.forward_speed)
                else:
                    self._wifi_seek(prox, (x, y, heading_degrees), target)
                return self.state
            self._wheels(-self.turn_speed, self.turn_speed)
            return self.state

        if front >= self.front_obstacle_threshold:
            self.state = 'TURN_CLEAR'
            self.clear_frames = 0
            self._wheels(-self.turn_speed, self.turn_speed)
            return self.state

        if target is not None:
            self._wifi_seek(prox, (x, y, heading_degrees), target)
        else:
            self.state = 'FORWARD'
            self._wheels(self.forward_speed, self.forward_speed)
        return self.state
