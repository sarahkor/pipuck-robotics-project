"""Standalone IR-proximity-based follower for the older Bluetooth mission.

The follower can rotate to locate an object in front, optionally signal
readiness through a callback, and adjust its wheel speeds using proximity
error and left/right sensor imbalance. It does not use Range & Bearing.

This module is not used by the current team_mission_rab.py mission.
"""

from pipuck import PiPuck
from time import sleep, time


class HandFollower:
    """Follow a nearby object using the PiPuck's IR proximity sensors.

    Args:
        puck: Active PiPuck hardware interface.
        ready_callback: Optional no-argument function invoked once when front
            detection is confirmed during startup search.
        search_before_follow: Begin by scanning for an object before following;
            otherwise start following immediately.

    Sensor readings serve as relative proximity indicators, not calibrated
    physical distances. Any sufficiently strong reading can be interpreted as
    the object to follow; no identity verification is performed.
    """

    def __init__(self, puck, ready_callback=None, search_before_follow=False):
        """Initialize search, tracking, and wheel-command smoothing parameters."""
        self.puck = puck

        self.ready_callback = ready_callback
        self.ready_sent = False

        if search_before_follow:
            self.state = "SEARCH_LEADER"
        else:
            self.state = "FOLLOW"

        self.target_distance = 150
        self.noise_threshold = 100

        self.startup_threshold = 80
        self.align_speed = 160
        self.ready_required_count = 2
        self.ready_count = 0
        self.search_direction = None

        self.search_speed = 250
        self.search_timeout = 2.5
        self.lost_time = None

        self.kp_dist = 2.8
        self.kd_dist = 0.5
        self.prev_dist_error = 0
        self.kp_steer = 1.0

        self.max_speed = 300

        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.smoothing_factor = 0.7

    def stop_motors(self):
        """Command both wheels to stop and transmit the command."""
        self.puck.epuck2.set_wheel_speeds(0, 0)
        self.puck.epuck2.update()

    def _send_ready_once(self):
        """Invoke the optional ready callback at most once per instance.

        The ready flag is set even when no callback was supplied.
        """
        if self.ready_sent:
            return

        if self.ready_callback is not None:
            print("Follower is ready. Sending READY signal...")
            self.ready_callback()

        self.ready_sent = True

    def _search_leader_step(self):
        """Take one IR-guided step to place the detected object in front.

        Two consecutive front detections trigger a stop, the readiness callback,
        and the transition from SEARCH_LEADER to FOLLOW.
        """
        frame = self.puck.epuck2.update()
        prox = list(frame.prox)

        # Use two groups of front-facing IR sensors to confirm a target.
        front_left = max(prox[7], prox[6])
        front_right = max(prox[0], prox[1])
        avg_front = (front_left + front_right) / 2.0

        # Side readings select the scan direction, not the target identity.
        left_side = max(prox[5], prox[6], prox[7])
        right_side = max(prox[0], prox[1], prox[2])

        if avg_front > self.startup_threshold:
            self.ready_count += 1
            print("Leader in front. Ready count: {}".format(self.ready_count))

            self.stop_motors()

            if self.ready_count >= self.ready_required_count:
                print("Leader confirmed in front. Sending READY and switching to FOLLOW.")
                self._send_ready_once()
                self.lost_time = None
                self.state = "FOLLOW"

            return

        self.ready_count = 0

        if left_side > self.startup_threshold or right_side > self.startup_threshold:
            if left_side > right_side:
                self.search_direction = "LEFT"
            else:
                self.search_direction = "RIGHT"

            print("Leader seen on side. Turning {} until front sensors see it.".format(
                self.search_direction
            ))

        if self.search_direction == "LEFT":
            self.puck.epuck2.set_wheel_speeds(-self.align_speed, self.align_speed)
            self.puck.epuck2.update()

        elif self.search_direction == "RIGHT":
            self.puck.epuck2.set_wheel_speeds(self.align_speed, -self.align_speed)
            self.puck.epuck2.update()

        else:
            # Fall back to a fixed scan direction until something is detected.
            print("No leader detected yet. Scanning...")
            self.puck.epuck2.set_wheel_speeds(self.align_speed, -self.align_speed)
            self.puck.epuck2.update()

    def _follow_step(self):
        """Update smoothed wheel commands from proximity measurements.

        When proximity is weak, advance for a limited interval and then command
        a stop. Otherwise regulate front proximity with PD terms, steer from
        left/right imbalance, and exponentially smooth wheel speeds.
        """
        frame = self.puck.epuck2.update()
        prox = list(frame.prox)

        left_sensor = max(prox[7], prox[6], prox[5])
        right_sensor = max(prox[0], prox[1], prox[2])
        avg_front = (left_sensor + right_sensor) / 2.0

        target_left = 0
        target_right = 0

        if avg_front < self.noise_threshold:
            if self.lost_time is None:
                self.lost_time = time()

            if time() - self.lost_time < self.search_timeout:
                target_left = self.search_speed
                target_right = self.search_speed
            else:
                target_left = 0
                target_right = 0

            self.prev_dist_error = 0

        else:
            self.lost_time = None

            dist_error = self.target_distance - avg_front
            dist_deriv = dist_error - self.prev_dist_error
            base_speed = (dist_error * self.kp_dist) + (dist_deriv * self.kd_dist)
            self.prev_dist_error = dist_error

            if 2 < base_speed < 20:
                base_speed = 20

            base_speed = max(min(base_speed, self.max_speed), -self.max_speed)

            steer = (left_sensor - right_sensor) * self.kp_steer
            target_left = base_speed - steer
            target_right = base_speed + steer

        target_left = max(min(target_left, self.max_speed), -self.max_speed)
        target_right = max(min(target_right, self.max_speed), -self.max_speed)

        self.current_left_speed = (
            self.smoothing_factor * target_left
            + ((1.0 - self.smoothing_factor) * self.current_left_speed)
        )

        self.current_right_speed = (
            self.smoothing_factor * target_right
            + ((1.0 - self.smoothing_factor) * self.current_right_speed)
        )

        self.puck.epuck2.set_wheel_speeds(
            int(self.current_left_speed),
            int(self.current_right_speed)
        )

    def run_mission(self):
        """Execute the search/follow state machine until externally interrupted.

        Call stop_motors() during caller-managed cleanup; this loop does not
        terminate on its own.
        """
        print("Follower mission started.")

        if self.state == "SEARCH_LEADER":
            print("State 1: SEARCH_LEADER")
            print("The follower will turn until its front sensors see the leader.")
            print("Then it sends READY and switches to old FOLLOW behavior.")
        else:
            print("State: FOLLOW")
            print("Running old follow behavior directly.")

        print("Press Ctrl+C to stop.")

        while True:
            if self.state == "SEARCH_LEADER":
                self._search_leader_step()

            elif self.state == "FOLLOW":
                self._follow_step()

            sleep(0.05)


if __name__ == "__main__":
    puck = PiPuck()
    follower = None

    with puck:
        try:
            follower = HandFollower(puck)
            follower.run_mission()

        except KeyboardInterrupt:
            print("\n[INTERRUPT] Manual stop detected. Halting robot...")

        finally:
            if follower:
                follower.stop_motors()
            else:
                puck.epuck2.set_wheel_speeds(0, 0)
                puck.epuck2.update()
