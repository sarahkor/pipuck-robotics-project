"""Run away from the closest object using proximity sensors.

Behavior:
- Continuously read the 8 proximity sensors.
- Compute an escape steering command opposite to the strongest/closest object.
- Immediately drive forward with differential steering (no separate turn phase).

-Notes:
- Real proximity sensor index order (0..7):
-   0=front-right (fr), 1=right-front (rf), 2=right (r), 3=back-right (br),
-   4=back-left (bl), 5=left (l), 6=left-front (lf), 7=front-left (fl).
- If your robot differs, tweak ANGLES_DEG and the UI mapping.
- Run this on the Pi-puck over an interactive terminal (local or SSH).
- Ensure I2C is enabled and the robot is powered on.

Optional args:
  --fwd <int>   Forward speed (ticks/s), default 300
  --turn <int>  Turn speed (ticks/s), default 300
  --min <val>   Minimum proximity to react; below this, the robot stops, default 50
"""

import sys
import time
import termios
import tty
import math
from typing import List

from pipuck import PiPuck


# Tunables (can be overridden by CLI)
FWD_SPEED = 300
TURN_SPEED = 300
MIN_PROX_TO_REACT = 100

def _configure_terminal() -> list:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old


def _restore_terminal(old_settings: list) -> None:
    try:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
    except Exception:
        pass

# Sensor bearings in degrees relative to the robot forward axis.
# Index order (0..7): fr, rf, r, br, bl, l, lf, fl
# Positive angles are to the left, negative to the right.
ANGLES_DEG = [
    -30.0,  # 0 fr  (front-right)
    -60.0,  # 1 rf  (right-front)
    -90.0,  # 2 r   (right)
    -150.0, # 3 br  (back-right)
    +150.0, # 4 bl  (back-left)
    +90.0,  # 5 l   (left)
    +60.0,  # 6 lf  (left-front)
    +30.0,  # 7 fl  (front-left)
] # type: List[float]


def _argmax(values: List[int]) -> int:
    mi = 0
    mv = values[0] if values else 0
    for i, v in enumerate(values):
        if v > mv:
            mv = v
            mi = i
    return mi


def _bar(val: int, vmax: int, width: int = 6) -> str:
    vmax = max(1, int(vmax))
    n = int(round(width * float(val) / float(vmax)))
    n = max(0, min(width, n))
    return "#" * n + "." * (width - n)


def _num(val: int, width: int = 6) -> str:
    return "{:>{w}d}".format(int(val), w=width)


def _render_prox_circle(prox: List[int], angle_deg: float, mode: str, left: int, right: int) -> None:
    """Render proximity in a circle-like ASCII layout with a status line."""
    b = [_num(p) for p in prox]

    # Clear screen and move cursor home
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.write("Proximity (circle): numeric values\n")
    # Circle-ish placement reflecting index order: 0 fr,1 rf,2 r,3 br,4 bl,5 l,6 lf,7 fl
    # Top row: front-left (7) and front-right (0)
    sys.stdout.write("\n         {}        {}\n".format(b[7], b[0]))
    # Next row: left-front (6) and right-front (1)
    sys.stdout.write("   {}                        {}\n".format(b[6], b[1]))
    # Middle row: left (5) and right (2)
    sys.stdout.write(" {}                              {}\n".format(b[5], b[2]))
    # Bottom row: back-left (4) and back-right (3)
    sys.stdout.write("   {}                        {}\n\n".format(b[4], b[3]))

    sys.stdout.write(
        "mode: {m:>5} | escape angle: {a:>6.1f}° | L={l:+d} R={r:+d}\n".format(
            m=mode, a=angle_deg, l=left, r=right
        )
    )
    sys.stdout.flush()


def main() -> None:
    global FWD_SPEED, TURN_SPEED, MIN_PROX_TO_REACT

    # Simple CLI parsing
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--fwd" and i + 1 < len(args):
            FWD_SPEED = int(args[i + 1]); i += 2
        elif args[i] == "--turn" and i + 1 < len(args):
            TURN_SPEED = int(args[i + 1]); i += 2
        elif args[i] == "--min" and i + 1 < len(args):
            MIN_PROX_TO_REACT = int(args[i + 1]); i += 2
        else:
            print(__doc__)
            print("Unknown/extra args:", " ".join(args[i:]))
            return

    print("Run-away behavior: steering away from closest object while driving forward.")

    old_tio = _configure_terminal()
    try:
        with PiPuck() as pi:
            bot = pi.epuck2

            # Optional: enable proximity calibration and disable onboard avoidance to control behavior here
            try:
                bot.calibrate_proximity(True)
            except Exception:
                pass

            # Ensure motors are stopped initially and push first update
            bot.set_wheel_speeds(0, 0)
            frame = bot.update()
            time.sleep(0.05)

            while True:
                frame = bot.update()
                prox = list(frame.prox)

                # If no meaningful signal, stop and wait
                pmax = max(prox) if prox else 0
                if pmax < MIN_PROX_TO_REACT:
                    left = right = 0
                    _render_prox_circle(prox, 0.0, "idle", left, right)
                    bot.set_wheel_speeds(0, 0)
                    time.sleep(0.05)
                    continue

                # Find closest object and compute escape angle as the opposite bearing
                idx = _argmax(prox)
                obj_bearing = ANGLES_DEG[idx]
                # Escape is 180° opposite of the obstacle bearing (wrapped to [-180, 180])
                angle_deg = ((obj_bearing + 180.0 + 360.0) % 360.0) - 180.0
                # Vector-based mixing: vx sets forward/back, vy sets steering
                rad = math.radians(angle_deg)
                vx = -math.cos(rad)  # +1 forward, -1 backward
                vy = -math.sin(rad)  # + left, - right
                # If this still steers the wrong way on your hardware, try: vy = -vy
                left = int(FWD_SPEED * vx - TURN_SPEED * vy)
                right = int(FWD_SPEED * vx + TURN_SPEED * vy)

                mode = "drive"

                bot.set_wheel_speeds(left, right)
                _render_prox_circle(prox, angle_deg, mode, left, right)

                # Short, regular loop
                time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors on exit
        try:
            with PiPuck() as pi:
                pi.epuck2.set_wheel_speeds(0, 0)
                pi.epuck2.update()
                time.sleep(0.05)
        except Exception:
            pass
        _restore_terminal(old_tio)
        # Move to next line after inline updates
        try:
            sys.stdout.write("\n")
            sys.stdout.flush()
        except Exception:
            pass


if __name__ == "__main__":
    main()
