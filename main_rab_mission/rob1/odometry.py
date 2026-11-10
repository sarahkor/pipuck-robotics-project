"""Local differential-drive odometry from cumulative wheel encoder steps.

The robot starts at (x, y) = (0, 0) millimeters, facing +Y. Positive X points
to its right, and clockwise-positive heading is stored internally in radians.
update() returns the heading in degrees. Estimates drift with wheel slip and
encoder errors; independent robots do not automatically share a global frame.
"""


import math


class Odometry:
    """Integrate e-puck2 wheel encoder differences into a local 2D pose.

    The motion model uses a midpoint-heading approximation for each encoder
    interval. Wheel geometry and encoder resolution are configured in __init__.
    """

    def __init__(self):
        """Initialize the local pose, wheel geometry, and previous encoder counts."""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # 0 radians = Facing Forward (+Y)

        # e-puck2 wheel geometry and encoder resolution.
        self.WHEEL_RADIUS_MM = 20.5
        self.WHEEL_BASE_MM = 53.0 
        self.STEPS_PER_REV = 1000.0

        # Nominal travel distance per encoder step, in millimeters.
        self.MM_PER_STEP = (2.0 * math.pi * self.WHEEL_RADIUS_MM) / self.STEPS_PER_REV

        self.prev_left = None
        self.prev_right = None

    def fuse_with_wifi_state(self, leader_wifi_x, leader_wifi_y, rab_range, rab_bearing_deg):
        """Blend the local XY estimate with a Wi-Fi/R&B-derived position estimate.

        Args:
            leader_wifi_x: Leading robot's X coordinate in its reported frame.
            leader_wifi_y: Leading robot's Y coordinate in its reported frame.
            rab_range: Raw R&B range measurement, scaled internally by 0.25.
            rab_bearing_deg: R&B bearing in degrees.

        This experimental method assumes compatible coordinate frames and a valid
        range calibration; it does not establish either condition. It adjusts X/Y
        by a 5 percent correction but leaves the current heading unchanged.
        """
        # Assumed R&B range-to-mm conversion (requires calibration).
        distance_mm = rab_range * 0.25 

        rab_bearing_rad = math.radians(rab_bearing_deg)
        global_angle = self.theta + rab_bearing_rad

        calculated_x = leader_wifi_x - (distance_mm * math.sin(global_angle))
        calculated_y = leader_wifi_y - (distance_mm * math.cos(global_angle))

        alpha = 0.95
        self.x = (alpha * self.x) + ((1.0 - alpha) * calculated_x)
        self.y = (alpha * self.y) + ((1.0 - alpha) * calculated_y)

    def update(self, left_steps, right_steps):
        """Integrate the latest cumulative 16-bit motor-step readings.

        Args:
            left_steps: Current cumulative left encoder count.
            right_steps: Current cumulative right encoder count.

        Returns:
            Tuple (x_mm, y_mm, heading_degrees) in the robot's local frame.

        The first call sets the encoder reference without moving the pose. Later
        calls unwrap 16-bit differences and integrate translation using the midpoint
        heading, while storing the heading internally in radians.
        """
        # The initial sample establishes the encoder baseline.
        if self.prev_left is None or self.prev_right is None:
            self.prev_left = left_steps
            self.prev_right = right_steps
            return self.x, self.y, math.degrees(self.theta)

        # Interpret differences modulo 2^16 as signed encoder motion.
        # Assumes fewer than 32768 steps between successive readings.
        d_left = (left_steps - self.prev_left) & 0xFFFF
        if d_left > 32767: 
            d_left -= 65536

        d_right = (right_steps - self.prev_right) & 0xFFFF
        if d_right > 32767: 
            d_right -= 65536

        self.prev_left = left_steps
        self.prev_right = right_steps

        dist_left = d_left * self.MM_PER_STEP
        dist_right = d_right * self.MM_PER_STEP

        d_center = (dist_left + dist_right) / 2.0

        # Positive angular displacement is clockwise from the +Y direction.
        d_theta = (dist_left - dist_right) / self.WHEEL_BASE_MM 

        # Integrate at the interval midpoint to reduce turn-induced error.
        theta_mid = self.theta + d_theta / 2.0

        self.x += d_center * math.sin(theta_mid)
        self.y += d_center * math.cos(theta_mid)
        self.theta += d_theta
        # Normalize internal radians to [0, 2*pi).
        self.theta = self.theta % (2 * math.pi)

        return self.x, self.y, math.degrees(self.theta)
