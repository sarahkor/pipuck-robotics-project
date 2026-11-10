"""Standalone right-wall-following experiment for a PiPuck robot.

The robot drives toward an obstacle, pivots until its right-side sensors
observe the wall, and then regulates proximity using a PID-shaped controller.
The integral gain is zero by default, so the active feedback is PD control.
The procedure is open-ended and must be interrupted to stop.

This is an experiment, not the follower used by team_mission_rab.py.
"""

from pipuck import PiPuck
from time import time, sleep

class WallFollower:
    """Approach, align with, and track a wall using proximity readings.

    Args:
        puck: Active PiPuck hardware interface.
        target_val: Desired right-side IR proximity reading (not a distance in mm).

    Wheel-speed commands and control gains are fixed experimental settings.
    """

    def __init__(self, puck, target_val):
        """Configure the hardware interface, speed, and controller state."""
        self.puck = puck
        self.target_val = target_val
        self.speed = 200
        self.kp = 0.05  # Proportional gain
        self.kd = 0.15          # Derivative gain
        self.previous_error = 0 # Previous error for the discrete derivative
        self.ki = 0     # Integral gain (disabled at zero)
        self.integral = 0   # Running error sum

    def _find_wall(self):
        """Drive forward until a front proximity reading reaches the target.

        The loop has no timeout; it assumes an obstacle will eventually be detected.
        """
        print("Driving straight until reaching wall...")
        frame = self.puck.epuck2.update()
        prox = list(frame.prox)
        front_val = max(prox[0], prox[7]) if prox else 0

        while front_val < self.target_val:
            self.puck.epuck2.set_wheel_speeds(self.speed, self.speed)
            frame = self.puck.epuck2.update()
            prox = list(frame.prox)
            front_val = max(prox[0], prox[7]) if prox else 0
            sleep (0.05)

        self.puck.epuck2.set_wheel_speeds(0, 0)
        frame = self.puck.epuck2.update()

    def _align_to_wall(self):
        """Pivot left until the right-side IR sensors detect the wall.

        The threshold is slightly below the desired tracking reading. This loop
        also has no timeout if the wall is not found.
        """
        print("Aligning to wall...")
        frame = self.puck.epuck2.update()
        prox = list(frame.prox)
        
        right_val = max(prox[2], prox[3]) if prox else 0

        turn_speed = 100  

        while right_val < self.target_val - 50:
            self.puck.epuck2.set_wheel_speeds(-turn_speed, turn_speed)
            
            frame = self.puck.epuck2.update()
            prox = list(frame.prox)
            
            right_val = max(prox[2], prox[3]) if prox else 0
            
            sleep(0.01) 

        self.puck.epuck2.set_wheel_speeds(0, 0)
        self.puck.epuck2.update()

    def _follow_wall(self):
        """Continuously track the wall using right-side IR feedback.

        Combine proportional, accumulated integral, and discrete derivative terms
        to generate differential wheel speeds. The integral is bounded to reduce
        windup, but its default gain is zero.
        """
        print("following wall...")
        self.puck.epuck2.set_wheel_speeds(self.speed, self.speed)
        while True: 
            frame = self.puck.epuck2.update()
            prox = list(frame.prox)
            right_val = max (prox[2], prox[3]) if prox else 0

            error = right_val - self.target_val

            self.integral += error
            # Bound the accumulated error to limit integral windup.
            self.integral = max(min(self.integral, 1000), -1000)

            derivative = error - self.previous_error
            correction = (error * self.kp) + (self.integral * self.ki) + (derivative * self.kd)

            left_speed = self.speed - correction
            right_speed = self.speed + correction

            left_speed = int(max(min(left_speed, 1000), -1000))
            right_speed = int(max(min(right_speed, 1000), -1000))
            self.puck.epuck2.set_wheel_speeds(left_speed, right_speed)
            self.previous_error = error

            sleep(0.05)


if __name__ == "__main__":
    print("Initializing PiPuck...")
    puck = PiPuck()

    with puck:
        print("Starting movement test...") 
        follower = WallFollower(puck, target_val=400)

        try:
            follower._find_wall()
            follower._align_to_wall()
            follower._follow_wall()
            
        except KeyboardInterrupt:
            print("\n Interrupted by User (Ctrl+C) ")
            
        finally:
            print("Stopping motors.")
            puck.epuck2.set_wheel_speeds(0, 0)
            puck.epuck2.update()
            print("Test completed safely.")
