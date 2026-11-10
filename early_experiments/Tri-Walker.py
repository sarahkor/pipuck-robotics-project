
"""Simple open-loop triangular patrol for a PiPuck robot.

The robot performs three straight driving segments, each followed by
an in-place turn. Movement is controlled by fixed time intervals
rather than sensor feedback or odometry.
"""

from pipuck import PiPuck
from time import time, sleep


class TriWalker:
    """Control a PiPuck robot along a three-leg patrol path.

    Args:
        puck: Initialized PiPuck hardware interface.
        speed: Wheel-speed command used for driving and turning.
        leg_time: Duration of each straight segment in seconds.
        turn_time: Duration of each in-place turn in seconds.
    """

    def __init__(self, puck, speed, leg_time, turn_time):
        self.puck = puck
        self.speed = speed
        self.leg_time = leg_time
        self.turn_time = turn_time

    def patrol(self):
        """Execute three straight segments and three turns, then stop.

        Each segment consists of driving forward for leg_time
        seconds, followed by an in-place turn for turn_time seconds.
        """
        for i in range(3):
            self._drive_leg()
            self._turn()
        self._stop()

    def _drive_leg(self):
        """Drive straight for the configured leg duration.

        Both wheels receive the same positive speed command.
        """
        print("Driving straight...")
        self.puck.epuck2.set_wheel_speeds(self.speed, self.speed)
        self.puck.epuck2.update()
        sleep(self.leg_time)

    def _turn(self):
        """Rotate in place for the configured turning duration.

        The wheels rotate in opposite directions, producing
        an approximately stationary rotation.
        """
        print("turning...")
        self.puck.epuck2.set_wheel_speeds(-self.speed, self.speed)
        self.puck.epuck2.update()
        sleep(self.turn_time)

    def _stop(self):
        """Stop both wheels and send the command to the robot."""
        print("Stopping robot...")
        self.puck.epuck2.set_wheel_speeds(0, 0)
        self.puck.epuck2.update()


if __name__ == "__main__":
    # TODO: implement imu in the PiPuck class
    print("Initializing PiPuck...")
    puck = PiPuck()

    with puck:
        print("Starting movement test...")
        start_time = time()
        walker = TriWalker(puck, speed=77, leg_time=10, turn_time=5.6)
        walker.patrol()
        print("Test completed")
