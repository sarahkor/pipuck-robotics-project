#!/usr/bin/env python3
"""Record a PiPuck camera locally, independently of robot control.

Edit CAMERA below to choose the front-facing or 360-degree camera.

Usage on each robot:
    cd ~/pipuck-walk
    python3 record_pov.py

An optional --device 0 or --device 1 overrides CAMERA.
Ctrl+C or SIGTERM closes the camera and finalizes the AVI file.
"""

import argparse
import datetime as dt
from pathlib import Path
import signal
import socket
import time

import cv2

# please notice that the device numbers are different with each initialization of the robot.
# check prior to running this script.
# ===================== CAMERA SELECTION =====================
CAMERA = "360"  # Choose "front" or "360"
CAMERA_DEVICES = {"front": 1, "360": 0}
# ============================================================

_running = True


def request_stop(_signum, _frame):
    global _running
    _running = False


def main():
    parser = argparse.ArgumentParser(description="Save local PiPuck POV footage")
    parser.add_argument("--device", type=int, default=None,
                        help="Override camera selection: 0 = front, 2 = 360")
    parser.add_argument("--fps", type=float, default=10.0,
                        help="Target recording rate and AVI playback rate")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--output-dir", default="recordings")
    args = parser.parse_args()
    if args.fps <= 0 or args.width <= 0 or args.height <= 0:
        parser.error("fps, width and height must be positive")
    if CAMERA not in CAMERA_DEVICES:
        parser.error('CAMERA must be "front" or "360"')

    device = CAMERA_DEVICES[CAMERA] if args.device is None else args.device
    camera_name = {0: "front", 2: "360"}.get(device, f"camera{device}")

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    hostname = socket.gethostname()
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = out_dir / f"{hostname}_{camera_name}_{timestamp}.avi"

    cap = cv2.VideoCapture(device)
    writer = None
    frames_written = 0
    try:
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera device {device}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

        ok, first = cap.read()
        if not ok or first is None:
            raise RuntimeError(f"Camera {device} opened but no frame arrived")
        height, width = first.shape[:2]
        writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"MJPG"),
                                 args.fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError(f"Cannot create video file: {out_path}")

        print(f"Recording {out_path} ({width}x{height} at target {args.fps:g} FPS)",
              flush=True)
        writer.write(first)
        frames_written += 1
        period = 1.0 / args.fps
        next_frame_at = time.monotonic() + period

        while _running:
            delay = next_frame_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if not _running:
                break
            ok, frame = cap.read()
            if not ok or frame is None:
                print("Warning: failed to read camera frame", flush=True)
            elif frame.shape[:2] == (height, width):
                writer.write(frame)
                frames_written += 1
            else:
                print("Warning: camera frame dimensions changed; skipping frame",
                      flush=True)
            next_frame_at += period
            # Do not spin rapidly in an attempt to catch up after a slow read.
            if next_frame_at < time.monotonic():
                next_frame_at = time.monotonic() + period
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        print(f"Recording stopped; {frames_written} frames saved to {out_path}",
              flush=True)


if __name__ == "__main__":
    main()
