"""Coordinate a PiPuck team using range-and-bearing (R&B) and Wi-Fi.

This is the shared mission entry point for two or three robots. Each PiPuck
identifies itself from its hostname, calibrates R&B, discovers its peers, selects
roles, and waits for the launcher's start marker before moving.

Formation modes:
    hub: One goal-seeking leader; all other robots follow that leader.
    line: A goal-seeking front leader, optional middle follower/relay, and tail.

R&B carries discovery, role information, movement flags and follower status;
filtered R&B range/bearing drives normal follower steering. Wi-Fi broadcasts
odometry and obstacle status and supplies displacement waypoints during R&B
loss. The robot's proximity sensors handle local obstacle avoidance; detailed
wheel control lives in the imported controller modules.

Timing: Four 120-ms radio slots per 480-ms cycle, with 10-ms guard intervals
at both edges of each slot. Each robot depends on sufficiently aligned clocks.

This module preserves the existing mission decisions and safety policy. It does
not guarantee globally consistent smart election, obstacle-free motion, or
accurate Wi-Fi displacement recovery when initial odometry axes differ.
"""

import argparse
import math
import os
import re
import signal
import socket
import statistics
import time
from collections import deque

from pipuck import PiPuck

from follow_rob import RangeBearingFollower
from range_and_bearing import RangeAndBearing
from range_and_bearing_filter import RangeAndBearingFilter
from range_and_bearing_protocol import (
    LeaderMotionMode,
    MessageType,
    decode_message,
    encode_message,
)
from reach_goal_rab import ReachGoalController
from wifi_state import WorldStateNetwork
from safe_recovery import WiFiDisplacementRecovery


TRANSMISSION_POWER = 0

CALIBRATION_TRIGGER = "RAB_CALIBRATE"
CALIBRATION_READY_MARKER = "RAB_CALIBRATION_READY"
START_TRIGGER = "RAB_TEAM_START"
ELECTION_READY_MARKER = "RAB_ELECTION_READY"

CALIBRATION_SYNC_DELAY = 1.0
DISCOVERY_TIMEOUT = 60.0
ELECTION_SETTLE_TIME = 4.0

SLOT_SECONDS = 0.12
SLOT_COUNT = 4
# Do not begin an R&B transmission within 10 ms of either slot boundary.
# Each 120 ms slot therefore has a 100 ms permitted send window.
SLOT_GUARD_SECONDS = 0.010
POLL_INTERVAL = 0.01
ROBOT_UPDATE_PERIOD = 0.05
STATUS_INTERVAL = 0.50

GOOD_PACKET_GAP = 1.00
STALE_PACKET_GAP = 1.75
LOST_PACKET_GAP = 2.00
CONTACT_TIMEOUT = 45.00

# Follower status values carried in HEARTBEAT.sequence.
FOLLOWER_STATUS_NORMAL = 0
FOLLOWER_STATUS_AVOIDING = 1
FOLLOWER_STATUS_HOLDING = 2

LEADER_NORMAL_SPEED = 200
LEADER_FOLLOWER_AVOID_SPEED = 130
LEADER_FOLLOWER_HOLD_SPEED = 140

# LEADER_MOVING uses its sequence field for motion mode (low four bits),
# formation-slow (bit 4), and formation-hold (bit 5).
LEADER_SEQUENCE_SLOW_FORMATION_FLAG = 16
LEADER_SEQUENCE_HOLD_FORMATION_FLAG = 32
LEADER_SEQUENCE_MODE_MASK = 0x0F
FOLLOWER_FORMATION_SLOW_SCALE = 0.65
FOLLOWER_FORMATION_SLOW_STALE_SCALE = 0.55

LINE_LOCAL_LEADER_NORMAL_SCALE = 0.85
LINE_LOCAL_LEADER_STALE_SCALE = 0.30
LINE_HOLD_FRONT_IF_CHILD_STALE_AFTER = STALE_PACKET_GAP
LINE_SLOW_FRONT_IF_CHILD_STALE_AFTER = GOOD_PACKET_GAP

# If any follower/local follower is actively bypassing an obstacle, the rest of
# the formation should wait. The avoiding robot keeps executing its own local
# obstacle routine; non-avoiding robots hold position.
FORMATION_HOLD_ON_AVOIDING = True

# Smart election timing and thresholds.
SMART_LOCAL_WINDOW = 11
SMART_REPORT_WINDOW = 9
SMART_MIN_LOCAL_SAMPLES = 7
SMART_MIN_REPORTS = 4
SMART_PEER_TIMEOUT = 4.0
SMART_ELECTION_TIMEOUT = 45.0
SMART_STABLE_REPORTS_REQUIRED = 4
SMART_DRAIN_SECONDS = 0.8
SMART_FINAL_ANNOUNCE_SECONDS = 8.0

# Line-mode role detection must not require all-to-all R&B reports. In a real
# line, the front and tail robots may not reliably see each other because the
# middle robot is between them. Each robot therefore decides from its own direct
# R&B view: peers in front / peers behind.
LINE_LOCAL_MIN_SAMPLES = 2
LINE_LOCAL_DECISION_AFTER_SECONDS = 8.0
LINE_LOCAL_STABLE_REQUIRED = 4

SMART_HEADING_TOLERANCE_DEG = 25.0
SMART_AHEAD_LIMIT_DEG = 70.0
SMART_BEHIND_LIMIT_DEG = 110.0

# Legacy bearing-report encoding helpers. Active smart elections currently
# use local measurements from beacons rather than these pairwise reports.
SMART_BEACON_TYPE = MessageType.LEADER_WAITING
SMART_BEARING_TYPE_BY_TARGET = {
    0: MessageType.HELLO,
    1: MessageType.ELECTION,
    2: MessageType.HEARTBEAT,
}
SMART_TARGET_BY_BEARING_TYPE = {
    message_type: target_id
    for target_id, message_type in SMART_BEARING_TYPE_BY_TARGET.items()
}

RGB_LED_INDICES = (2, 4, 6, 8)

_stop_requested = False


def request_stop(signum, frame):
    """Request termination when SIGINT or SIGTERM arrives.
    
    Sets a shared flag checked by the long-running mission loops; it does not
    itself command the motors to stop. Motor cleanup occurs in ``main``.
    """
    global _stop_requested
    _stop_requested = True


def parse_args():
    """Parse required team size and optional mission settings.
    
    Returns an argparse namespace containing the formation mode, election mode,
    and leader goal coordinates in millimeters. This is run on every PiPuck.
    """
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--team-size",
        type=int,
        choices=(2, 3),
        required=True,
    )
    parser.add_argument(
        "--target-x",
        type=float,
        default=0.0,
        help="Goal X in millimeters; positive is to the robot's right.",
    )
    parser.add_argument(
        "--target-y",
        type=float,
        default=400.0,
        help="Goal Y in millimeters; positive is forward.",
    )

    parser.add_argument(
        "--smart-election",
        action="store_true",
        help=(
            "Use stationary bearing-based smart election. "
            "If omitted, use old/simple election: smallest robot ID is leader."
        ),
    )

    parser.add_argument(
        "--formation-mode",
        choices=("hub", "line"),
        default="hub",
        help=(
            "hub = old behavior: all followers follow the elected leader. "
            "line = already-formed chain: front robot is leader, middle robot "
            "is local leader, back robot follows the middle robot."
        ),
    )

    return parser.parse_args()


def robot_id_from_hostname():
    """Extract the numeric robot ID from the end of the local hostname.
    
    For example, ``pipuck2`` maps to ID 2. Rejects missing or out-of-range IDs.
    """
    match = re.search(r"(\d+)$", socket.gethostname())

    if match is None:
        raise RuntimeError("Cannot parse robot ID from hostname")

    robot_id = int(match.group(1))

    if not 0 <= robot_id <= 3:
        raise RuntimeError("Robot ID must be in 0..3")

    return robot_id


def cycle_and_slot(now):
    """Map an absolute clock timestamp to a radio cycle and slot index.
    
    The four-slot cycle repeats every ``SLOT_COUNT * SLOT_SECONDS`` seconds.
    All robots compute this independently, so clock synchronization matters.
    """
    cycle_seconds = SLOT_SECONDS * SLOT_COUNT
    cycle = int(math.floor(now / cycle_seconds))
    phase = now - cycle * cycle_seconds
    slot = int(phase / SLOT_SECONDS)
    return cycle, slot


def set_all_rgb(puck, red, green, blue):
    """Set every configured RGB LED to one color and apply the update.
    
    The color is used to show calibration and elected mission roles.
    """
    for index in RGB_LED_INDICES:
        puck.epuck2.set_rgb_led(index, red, green, blue)

    puck.epuck2.update()


def remove_marker(path):
    """Delete a launcher marker file if present, ignoring OS errors."""
    try:
        if os.path.exists(path):
            os.remove(path)
    except OSError:
        pass


def safe_stop(puck):
    """Best-effort hardware cleanup: stop wheels and switch off LEDs.
    
    Cleanup failures are printed instead of propagated; this runs from the
    ``main`` finally block even if a mission function raises an exception.
    """
    try:
        puck.epuck2.set_wheel_speeds(0, 0)

        for index in RGB_LED_INDICES:
            puck.epuck2.set_rgb_led(index, 0, 0, 0)

        puck.epuck2.update()
    except Exception as error:
        print("Cleanup warning: {}".format(error))


def read_mission_packet(
    link,
    allowed_sender_ids,
    allowed_message_types,
    max_reads=6,
):
    """Read up to ``max_reads`` R&B packets and select an allowed message.
    
    Only the supplied sender IDs and message types are accepted. Invalid packets
    and packets for other links are skipped. Returns ``(reading, message)`` or
    ``(None, None)`` if nothing eligible is obtained in the read budget.
    """
    allowed_sender_ids = {int(value) for value in allowed_sender_ids}
    allowed_message_types = set(allowed_message_types)

    for _ in range(int(max_reads)):
        reading, message = link.read()

        if reading is None and message is None:
            return None, None

        if message is None:
            continue

        if int(message.sender_id) not in allowed_sender_ids:
            continue

        if message.message_type not in allowed_message_types:
            continue

        return reading, message

    return None, None


class RabLink:
    """R&B packet transport with timed transmit slots and error counters.
    
    Outgoing messages use the protocol encoder. Each slot can be used at most
    once per cycle by this instance; the slot margins restrict when sending
    begins, not the hardware transmit completion time.
    """
    def __init__(self, rab, robot_id):
        """Store the R&B device and sender ID; initialize sequence and counters."""
        self.rab = rab
        self.robot_id = int(robot_id)
        self.sequence = 0
        self.last_sent_key = None
        self.i2c_errors = 0
        self.crc_errors = 0

    def send_in_slots(
        self,
        message_type,
        slots,
        sequence_override=None,
    ):
        """Send at most one encoded packet in an authorized, guarded time slot.
        
        ``slots`` contains the allowed slot indices. The optional sequence override
        also carries status or motion flags in some message types. Return True only
        when ``rab.send`` succeeds; count I2C errors without raising them here.
        """
        now = time.time()
        cycle, slot = cycle_and_slot(now)
        key = (cycle, slot)

        if slot not in slots or key == self.last_sent_key:
            return False

        # Keep 10 ms clear on both sides of every 120 ms transmission slot.
        # Use the same clock sample as cycle_and_slot() to avoid crossing a
        # boundary between separate time.time() calls.
        cycle_start = cycle * (SLOT_SECONDS * SLOT_COUNT)
        slot_elapsed = now - (cycle_start + slot * SLOT_SECONDS)
        if (slot_elapsed < SLOT_GUARD_SECONDS
                or slot_elapsed >= SLOT_SECONDS - SLOT_GUARD_SECONDS):
            return False

        sequence = (
            self.sequence
            if sequence_override is None
            else int(sequence_override)
        )

        packet = encode_message(
            message_type,
            sender_id=self.robot_id,
            sequence=sequence,
        )

        try:
            self.rab.send(packet)
        except OSError:
            self.i2c_errors += 1
            return False

        if sequence_override is None:
            self.sequence = (self.sequence + 1) % 128

        self.last_sent_key = key
        return True

    def read(self):
        """Read and decode one R&B measurement and its compact message.
        
        Return ``(None, None)`` when no data or an I2C read error occurs; return
        ``(reading, None)`` for data that fails decoding, counting that error.
        """
        try:
            reading = self.rab.read()
        except OSError:
            self.i2c_errors += 1
            return None, None

        if reading is None:
            return None, None

        message = decode_message(reading.data)

        if message is None:
            self.crc_errors += 1

        return reading, message

class DummyFrame:
    """Small frame-shaped holder for a proximity array.
    
    This class is retained for compatibility but is not instantiated elsewhere
    in this module. Its motor steps are fixed at zero.
    """
    def __init__(self, prox):
        """Store proximity values and expose zero-valued motor steps."""
        self.motor_steps = (0, 0)
        self.prox = prox



def election_slots(robot_id):
    """Use the robot's numeric ID as its discovery/election slot."""
    return {int(robot_id)}


def mission_slots(robot_id, leader_id, team_ids):
    """Assign hub-mode R&B transmit slots for two or three robots.
    
    For two robots, the leader uses slots 0/2 and the follower 1/3.
    For three robots, the leader uses 0/2 and the sorted follower IDs use 1/3.
    """
    team_ids = sorted(int(value) for value in team_ids)

    if len(team_ids) == 2:
        if robot_id == leader_id:
            return {0, 2}
        return {1, 3}

    follower_ids = [
        value
        for value in team_ids
        if value != leader_id
    ]

    if robot_id == leader_id:
        return {0, 2}

    follower_rank = follower_ids.index(robot_id)
    return {1} if follower_rank == 0 else {3}


def line_mission_slots(robot_id, team_ids):
    """Assign line-mode transmit slots by sorted robot ID, not line role.
    
    For three robots, the middle *sorted ID* gets slots 1/3; the first and
    last sorted IDs get slots 0 and 2. This is independent of smart role election.
    """
    team_ids = sorted(int(value) for value in team_ids)
    index = team_ids.index(int(robot_id))

    if len(team_ids) == 2:
        return {0} if index == 0 else {2}

    # For three robots, the second sorted ID gets two slots to relay.
    # This schedule is ID-based, even when elected line roles are different.
    return {1, 3} if index == 1 else {index}


def wait_for_calibration_trigger(robot_id):
    """Publish readiness and wait for the launcher's calibration marker.
    
    Once the marker exists, delay by the configured calibration sync interval.
    Return False only when a stop signal arrives while waiting.
    """
    with open(CALIBRATION_READY_MARKER, "w") as marker:
        marker.write("robot_id={}\n".format(robot_id))

    print("Ready for synchronized ambient calibration.")

    while (
        not _stop_requested
        and not os.path.exists(CALIBRATION_TRIGGER)
    ):
        time.sleep(POLL_INTERVAL)

    if _stop_requested:
        return False

    time.sleep(CALIBRATION_SYNC_DELAY)
    return True


def discover_team(robot_id, team_size, link):
    """Discover team membership from R&B ELECTION announcements.
    
    Each robot advertises IDs it knows, so membership information can propagate
    through intermediate peers. Return the sorted selected IDs once team size
    has been stable for the settlement period; raise on discovery timeout.
    """
    seen_ids = {robot_id}
    start = time.monotonic()
    last_report = 0.0
    advertise_index = 0
    complete_since = None

    print("R&B election: discovering {} robots...".format(team_size))

    while (
        not _stop_requested
        and time.monotonic() - start < DISCOVERY_TIMEOUT
    ):
        advertised_ids = sorted(seen_ids)
        advertised_id = advertised_ids[
            advertise_index % len(advertised_ids)
        ]

        sent = link.send_in_slots(
            MessageType.ELECTION,
            election_slots(robot_id),
            sequence_override=advertised_id,
        )

        if sent:
            advertise_index += 1

        reading, message = link.read()

        if (
            reading is not None
            and message is not None
            and message.message_type == MessageType.ELECTION
        ):
            candidates = (
                int(message.sender_id),
                int(message.sequence),
            )

            for candidate_id in candidates:
                if not 0 <= candidate_id <= 3:
                    continue

                if candidate_id not in seen_ids:
                    seen_ids.add(candidate_id)
                    print(
                        "DISCOVERED id={} via sender={} "
                        "bearing={:+.2f} range={} peak={}".format(
                            candidate_id,
                            int(message.sender_id),
                            reading.bearing_degrees,
                            reading.range_raw,
                            reading.max_peak,
                        )
                    )

        now = time.monotonic()

        if now - last_report >= 2.0:
            print("Election IDs: {}".format(sorted(seen_ids)))
            last_report = now

        if len(seen_ids) >= team_size:
            if complete_since is None:
                complete_since = now
                print(
                    "Election membership complete; relaying for {:.1f}s.".format(
                        ELECTION_SETTLE_TIME
                    )
                )
            elif now - complete_since >= ELECTION_SETTLE_TIME:
                return sorted(seen_ids)[:team_size]
        else:
            complete_since = None

        time.sleep(POLL_INTERVAL)

    raise RuntimeError(
        "R&B election timed out. Expected {}, saw {}".format(
            team_size,
            sorted(seen_ids),
        )
    )


def wait_for_start_trigger(robot_id, link, team_ids):
    """Continue membership advertisements while waiting for the start marker.
    
    This keeps R&B active but does not start movement. The stop flag also ends
    the wait; the caller checks it before dispatching the mission.
    """
    print("Election complete. Waiting for the user start trigger.")

    advertised_ids = sorted(int(value) for value in team_ids)
    advertise_index = 0

    while not _stop_requested and not os.path.exists(START_TRIGGER):
        advertised_id = advertised_ids[
            advertise_index % len(advertised_ids)
        ]

        sent = link.send_in_slots(
            MessageType.ELECTION,
            election_slots(robot_id),
            sequence_override=advertised_id,
        )

        if sent:
            advertise_index += 1

        link.read()
        time.sleep(POLL_INTERVAL)


def wrap_degrees(angle):
    """Normalize an angle in degrees to the half-open interval [-180, 180)."""
    return ((float(angle) + 180.0) % 360.0) - 180.0


def circular_median(angles):
    """Compute a median of bearings while accounting for angle wraparound.
    
    Unwrap each sample relative to the last sample, take the scalar median,
    then normalize the result. Raises ValueError for an empty sequence.
    """
    if not angles:
        raise ValueError("angles must not be empty")

    reference = float(angles[-1])
    unwrapped = [
        reference + wrap_degrees(float(value) - reference)
        for value in angles
    ]
    return wrap_degrees(statistics.median(unwrapped))


def relative_heading(bearing_i_to_j, bearing_j_to_i):
    """Estimate relative orientation from reciprocal bearing measurements.
    
    Both bearings are in degrees; the returned normalized difference reflects
    how the two robots' local headings relate under the R&B bearing convention.
    """
    return wrap_degrees(
        180.0 + float(bearing_i_to_j) - float(bearing_j_to_i)
    )


def classify_relative_position(bearing):
    """Classify a bearing as AHEAD, BEHIND or SIDE using configured cones."""
    absolute = abs(wrap_degrees(bearing))

    if absolute <= SMART_AHEAD_LIMIT_DEG:
        return "AHEAD"
    if absolute >= SMART_BEHIND_LIMIT_DEG:
        return "BEHIND"
    return "SIDE"


def encode_smart_bearing_degrees(angle):
    """Quantize a bearing in degrees into a clamped seven-bit payload."""
    normalized = wrap_degrees(angle)
    payload = int(round((normalized + 180.0) * 127.0 / 360.0))
    return max(0, min(127, payload))


def decode_smart_bearing_degrees(payload):
    """Decode a seven-bit bearing payload back to approximate degrees.
    
    This helper is retained but has no caller within this module.
    """
    payload = int(payload)
    if not 0 <= payload <= 127:
        raise ValueError("payload must be between 0 and 127")
    return wrap_degrees((payload * 360.0 / 127.0) - 180.0)


def smart_bearing_message_type(target_id):
    """Choose the temporary election report message type for a target ID.
    
    Only the target IDs configured in SMART_BEARING_TYPE_BY_TARGET are supported.
    """
    target_id = int(target_id)
    if target_id not in SMART_BEARING_TYPE_BY_TARGET:
        raise RuntimeError(
            "smart election currently supports target IDs {} only".format(
                sorted(SMART_BEARING_TYPE_BY_TARGET.keys())
            )
        )
    return SMART_BEARING_TYPE_BY_TARGET[target_id]


def smart_bearing_target_id(message_type):
    """Map a temporary bearing-report message type to its target ID.
    
    Retained for the older report-exchange path; not called elsewhere here.
    """
    return SMART_TARGET_BY_BEARING_TYPE.get(message_type)


def analyze_smart_team(team_ids, bearing_matrix):
    """Analyze reciprocal peer bearings for a proposed global leader.
    
    Build heading and ahead-of relationships, reject misaligned or inconsistent
    pairs, and elect the smallest-ID front candidate when the checks pass.
    This complete pairwise analysis is not used by the active local hub election.
    """
    team_ids = sorted(int(value) for value in team_ids)
    pair_deltas = {}
    pair_views = {}
    aligned = True

    ahead_of = {robot_id: set() for robot_id in team_ids}
    invalid_pairs = []

    for index, robot_i in enumerate(team_ids):
        for robot_j in team_ids[index + 1:]:
            bearing_i_j = float(bearing_matrix[(robot_i, robot_j)])
            bearing_j_i = float(bearing_matrix[(robot_j, robot_i)])

            delta = relative_heading(bearing_i_j, bearing_j_i)
            pair_deltas[(robot_i, robot_j)] = delta
            if abs(delta) > SMART_HEADING_TOLERANCE_DEG:
                aligned = False

            view_i = classify_relative_position(bearing_i_j)
            view_j = classify_relative_position(bearing_j_i)
            pair_views[(robot_i, robot_j)] = (view_i, view_j)

            if view_i == "AHEAD" and view_j == "BEHIND":
                ahead_of[robot_i].add(robot_j)
            elif view_i == "BEHIND" and view_j == "AHEAD":
                ahead_of[robot_j].add(robot_i)
            elif view_i == "SIDE" and view_j == "SIDE":
                pass
            else:
                invalid_pairs.append((robot_i, robot_j, view_i, view_j))

    anchor = min(team_ids)
    headings = {anchor: 0.0}
    for robot_id in team_ids:
        if robot_id == anchor:
            continue
        headings[robot_id] = relative_heading(
            bearing_matrix[(anchor, robot_id)],
            bearing_matrix[(robot_id, anchor)],
        )

    ahead_count = {
        robot_id: len(ahead_of[robot_id])
        for robot_id in team_ids
    }

    if not aligned:
        return {
            "aligned": False,
            "headings": headings,
            "pair_deltas": pair_deltas,
            "pair_views": pair_views,
            "ahead_of": ahead_of,
            "ahead_count": ahead_count,
            "leader_id": None,
            "reason": "team headings are not aligned",
            "invalid_pairs": invalid_pairs,
        }

    if invalid_pairs:
        return {
            "aligned": True,
            "headings": headings,
            "pair_deltas": pair_deltas,
            "pair_views": pair_views,
            "ahead_of": ahead_of,
            "ahead_count": ahead_count,
            "leader_id": None,
            "reason": "inconsistent pairwise position measurements",
            "invalid_pairs": invalid_pairs,
        }

    minimum_ahead = min(ahead_count.values())
    if minimum_ahead != 0:
        return {
            "aligned": True,
            "headings": headings,
            "pair_deltas": pair_deltas,
            "pair_views": pair_views,
            "ahead_of": ahead_of,
            "ahead_count": ahead_count,
            "leader_id": None,
            "reason": "cyclic front-order measurements",
            "invalid_pairs": invalid_pairs,
        }

    front_candidates = [
        robot_id
        for robot_id in team_ids
        if ahead_count[robot_id] == 0
    ]
    leader_id = min(front_candidates)

    if len(front_candidates) == 1:
        reason = "unique front-most robot"
    else:
        reason = "front-position tie; smaller ID wins"

    return {
        "aligned": True,
        "headings": headings,
        "pair_deltas": pair_deltas,
        "pair_views": pair_views,
        "ahead_of": ahead_of,
        "ahead_count": ahead_count,
        "leader_id": leader_id,
        "reason": reason,
        "invalid_pairs": invalid_pairs,
    }


def format_smart_facing(headings):
    """Format a robot-to-heading mapping for election diagnostics.
    
    This helper currently has no caller within this module.
    """
    parts = []
    for robot_id in sorted(headings):
        parts.append("pipuck{}={:+.1f}deg".format(robot_id, headings[robot_id]))
    return " ".join(parts)


def format_smart_positions(team_ids, ahead_of):
    """Format pairwise ahead-of relationships for election diagnostics.
    
    This helper currently has no caller within this module.
    """
    parts = []
    for robot_id in sorted(team_ids):
        robots_ahead = sorted(ahead_of[robot_id])
        if not robots_ahead:
            description = "front-most/tied-front"
        else:
            description = "behind {}".format(
                ",".join("pipuck{}".format(value) for value in robots_ahead)
            )
        parts.append("pipuck{}:{}".format(robot_id, description))
    return " | ".join(parts)


def drain_link(link, duration_seconds):
    """Discard R&B packets for a bounded period between protocol phases.
    
    Used to reduce interference from discovery/election messages during the
    subsequent readiness and mission phases.
    """
    end = time.monotonic() + float(duration_seconds)
    while not _stop_requested and time.monotonic() < end:
        link.read()
        time.sleep(POLL_INTERVAL)


def keep_sharing_smart_reports(robot_id, peer_ids, link, local_angles, duration_seconds):
    """Continue transmitting encoded pairwise bearing reports for a period.
    
    This helper belongs to an older report-exchange approach and has no active
    caller in this module. It is deliberately left intact.
    """
    if not peer_ids:
        return

    print(
        "SMART FINAL: continuing to share bearing reports for {:.1f}s "
        "so every robot can finish election.".format(duration_seconds)
    )

    end = time.monotonic() + float(duration_seconds)
    report_index = 0

    while not _stop_requested and time.monotonic() < end:
        target_id = peer_ids[report_index % len(peer_ids)]

        if len(local_angles[target_id]) > 0:
            local_median = circular_median(list(local_angles[target_id]))
            payload = encode_smart_bearing_degrees(local_median)
            message_type = smart_bearing_message_type(target_id)
            sent = link.send_in_slots(
                message_type,
                election_slots(robot_id),
                sequence_override=payload,
            )
            if sent:
                report_index += 1

        link.read()
        time.sleep(POLL_INTERVAL)


def run_smart_election(robot_id, team_ids, link):
    """Elect a hub-mode leader from each robot's own R&B front-cone view.
    
    Send beacons, aggregate visible peers' bearings, and choose self when no
    visible peer is ahead; otherwise choose the smallest ID among peers ahead.
    Return after a stable local decision or raise on timeout. This is not a
    consensus protocol, so different robots can choose different leaders.
    """
    team_ids = sorted(int(value) for value in team_ids)
    peer_ids = [value for value in team_ids if value != robot_id]

    print("SMART ELECTION (HUB): Checking local front cone...")
    drain_link(link, SMART_DRAIN_SECONDS)

    local_angles = {
        peer_id: deque(maxlen=SMART_LOCAL_WINDOW)
        for peer_id in peer_ids
    }
    last_seen = {peer_id: None for peer_id in peer_ids}
    
    stable_reports = 0
    previous_leader = None
    start = time.monotonic()
    last_status = 0.0

    while not _stop_requested and time.monotonic() - start < SMART_ELECTION_TIMEOUT:
        now = time.monotonic()

        # Keep sending a simple beacon so neighbors can measure our bearing
        link.send_in_slots(
            SMART_BEACON_TYPE,
            election_slots(robot_id),
            sequence_override=0,
        )

        reading, message = link.read()
        if (
            reading is not None
            and message is not None
            and int(message.sender_id) in peer_ids
            and reading.has_signal_measurement
        ):
            sender_id = int(message.sender_id)
            last_seen[sender_id] = now
            local_angles[sender_id].append(float(reading.bearing_degrees))

        if now - last_status >= STATUS_INTERVAL:
            last_status = now
            
            visible_peers = [p for p in peer_ids if last_seen[p] is not None and len(local_angles[p]) >= SMART_MIN_LOCAL_SAMPLES]
            stale_peers = [p for p in peer_ids if last_seen[p] is not None and now - last_seen[p] > SMART_PEER_TIMEOUT]
            
            if stale_peers:
                print("SMART LOST: stale peers={} ; waiting...".format(stale_peers))
                previous_leader = None
                stable_reports = 0
                time.sleep(POLL_INTERVAL)
                continue
                
            if not visible_peers:
                print("HUB ELECTION: Waiting to physically see any peers...")
                time.sleep(POLL_INTERVAL)
                continue

            front_peers = []
            for p in visible_peers:
                bearing = circular_median(list(local_angles[p]))
                if classify_relative_position(bearing) == "AHEAD":
                    front_peers.append(p)
            
            if len(front_peers) == 0:
                leader_id = robot_id
                print("HUB ELECTION CANDIDATE: No one in front. I am LEADER.")
            else:
                leader_id = min(front_peers)
                print("HUB ELECTION CANDIDATE: Robot(s) in front. I am FOLLOWER to pipuck{}.".format(leader_id))

            if leader_id == previous_leader:
                stable_reports += 1
            else:
                previous_leader = leader_id
                stable_reports = 1

            if stable_reports >= SMART_STABLE_REPORTS_REQUIRED:
                print("HUB ELECTION FINAL: Leader is pipuck{}.".format(leader_id))
                drain_link(link, SMART_DRAIN_SECONDS)
                return int(leader_id)

        time.sleep(POLL_INTERVAL)

    raise RuntimeError("Hub smart election timed out.")

def line_local_peer_summary(local_angles, local_ranges):
    """Describe locally observed line neighbors for status logs.
    
    Include each peer's median bearing, optional median raw range, and sample
    count. Raw R&B range values are not assumed to be millimeters.
    """
    parts = []
    for peer_id in sorted(local_angles):
        if len(local_angles[peer_id]) < 1:
            parts.append("{}:samples=0".format(robot_name(peer_id)))
            continue

        bearing = circular_median(list(local_angles[peer_id]))
        range_value = None
        if len(local_ranges.get(peer_id, [])) > 0:
            range_value = statistics.median(list(local_ranges[peer_id]))
        position = classify_relative_position(bearing)

        if range_value is None:
            parts.append(
                "{}:{} bearing={:+.1f} samples={}".format(
                    robot_name(peer_id),
                    position,
                    bearing,
                    len(local_angles[peer_id]),
                )
            )
        else:
            parts.append(
                "{}:{} bearing={:+.1f} range={:.0f} samples={}".format(
                    robot_name(peer_id),
                    position,
                    bearing,
                    range_value,
                    len(local_angles[peer_id]),
                )
            )

    if not parts:
        return "none"
    return " | ".join(parts)


def choose_nearest_peer(candidates):
    """Choose the peer ID with minimum observed raw range.
    
    Candidates are ``(range, peer_id, bearing)`` tuples; missing ranges sort
    last and peer ID breaks ties. Returns None when no candidates exist.
    """
    if not candidates:
        return None

    candidates = sorted(
        candidates,
        key=lambda item: (
            float("inf") if item[0] is None else float(item[0]),
            int(item[1]),
        ),
    )
    return int(candidates[0][1])


def analyze_local_line_role(robot_id, team_ids, local_angles, local_ranges):
    """Infer this robot's line role from its own direct R&B observations.
    
    Classify peers as in front, behind, or to the side; choose the nearest
    front and child candidates. Return a role/front/child dictionary and a
    reason, or ``(None, reason)`` when observations are inconclusive.
    """
    robot_id = int(robot_id)
    front_candidates = []
    behind_candidates = []
    side_peers = []
    usable_peers = []

    for peer_id in sorted(int(value) for value in team_ids if int(value) != robot_id):
        if len(local_angles.get(peer_id, [])) < LINE_LOCAL_MIN_SAMPLES:
            continue

        bearing = circular_median(list(local_angles[peer_id]))
        range_value = None
        if len(local_ranges.get(peer_id, [])) > 0:
            range_value = statistics.median(list(local_ranges[peer_id]))

        position = classify_relative_position(bearing)
        usable_peers.append(peer_id)

        if position == "AHEAD":
            front_candidates.append((range_value, peer_id, bearing))
        elif position == "BEHIND":
            behind_candidates.append((range_value, peer_id, bearing))
        else:
            side_peers.append((peer_id, bearing, range_value))

    if not usable_peers:
        return None, "no direct peer samples yet"

    front_id = choose_nearest_peer(front_candidates)
    child_id = choose_nearest_peer(behind_candidates)

    if front_id is None and child_id is not None:
        return {
            "role": "LEADER",
            "front_id": None,
            "child_id": child_id,
        }, "no peer in front; closest peer behind is {}".format(robot_name(child_id))

    if front_id is not None and child_id is not None:
        return {
            "role": "LOCAL_LEADER",
            "front_id": front_id,
            "child_id": child_id,
        }, "peer in front {} and peer behind {}".format(
            robot_name(front_id),
            robot_name(child_id),
        )

    if front_id is not None and child_id is None:
        return {
            "role": "FOLLOWER",
            "front_id": front_id,
            "child_id": None,
        }, "closest peer in front is {}; no peer behind".format(robot_name(front_id))

    return None, "only SIDE peers seen; line is not aligned enough"


def line_decision_signature(decision):
    """Extract the role/front/child triple used to test decision stability."""
    if decision is None:
        return None
    return (
        decision.get("role"),
        decision.get("front_id"),
        decision.get("child_id"),
    )


def run_line_role_election(robot_id, team_ids, link):
    """Collect stationary R&B samples and choose this robot's line role.
    
    Use local bearings and raw ranges rather than requiring all-to-all contact.
    A consistent role decision must persist across iterations and the minimum
    observation time; a last known decision can be returned at timeout.
    """
    robot_id = int(robot_id)
    team_ids = sorted(int(value) for value in team_ids)
    peer_ids = [value for value in team_ids if value != robot_id]

    print("LINE FORMATION: stationary local front/middle/back role detection.")
    print("LINE FORMATION: robots must already be physically arranged in a line.")
    print("LINE FORMATION: all-to-all reports are NOT required in this version.")
    print("LINE FORMATION: team_ids={}".format(team_ids))
    print(
        "LINE FORMATION: front cone +/-{:.1f}deg, behind cone >={:.1f}deg; motors remain stopped.".format(
            SMART_AHEAD_LIMIT_DEG,
            SMART_BEHIND_LIMIT_DEG,
        )
    )

    drain_link(link, SMART_DRAIN_SECONDS)

    local_angles = {
        peer_id: deque(maxlen=SMART_LOCAL_WINDOW)
        for peer_id in peer_ids
    }
    local_ranges = {
        peer_id: deque(maxlen=SMART_LOCAL_WINDOW)
        for peer_id in peer_ids
    }
    last_seen = {peer_id: None for peer_id in peer_ids}

    last_status = 0.0
    previous_signature = None
    stable_reports = 0
    final_decision = None
    final_reason = None
    start = time.monotonic()

    while not _stop_requested and time.monotonic() - start < SMART_ELECTION_TIMEOUT:
        now = time.monotonic()

        # Keep sending a simple beacon so neighbors can measure our bearing.
        link.send_in_slots(
            SMART_BEACON_TYPE,
            election_slots(robot_id),
            sequence_override=0,
        )

        reading, message = link.read()
        if (
            reading is not None
            and message is not None
            and int(message.sender_id) in peer_ids
            and reading.has_signal_measurement
        ):
            sender_id = int(message.sender_id)
            last_seen[sender_id] = now
            local_angles[sender_id].append(float(reading.bearing_degrees))
            local_ranges[sender_id].append(float(reading.range_raw))

        decision, reason = analyze_local_line_role(
            robot_id,
            team_ids,
            local_angles,
            local_ranges,
        )
        signature = line_decision_signature(decision)
        if signature is not None and signature == previous_signature:
            stable_reports += 1
        else:
            previous_signature = signature
            stable_reports = 1 if signature is not None else 0

        if signature is not None:
            final_decision = decision
            final_reason = reason

        if now - last_status >= STATUS_INTERVAL:
            last_status = now
            missing_peers = [
                peer_id
                for peer_id in peer_ids
                if last_seen[peer_id] is None
            ]
            stale_peers = [
                peer_id
                for peer_id in peer_ids
                if (
                    last_seen[peer_id] is not None
                    and now - last_seen[peer_id] > SMART_PEER_TIMEOUT
                )
            ]

            print(
                "LINE LOCAL VIEW: {}".format(
                    line_local_peer_summary(local_angles, local_ranges)
                )
            )

            if stale_peers:
                print("LINE LOCAL: stale peers={} ; continuing with direct local evidence.".format(stale_peers))

            if signature is None:
                print(
                    "LINE LOCAL COLLECTING: missing_peers={} reason={} stable=0/{} rab_errors={} crc_errors={}".format(
                        missing_peers,
                        reason,
                        LINE_LOCAL_STABLE_REQUIRED,
                        link.i2c_errors,
                        link.crc_errors,
                    )
                )
            else:
                print(
                    "LINE LOCAL CANDIDATE: role={} front={} child={} stable={}/{} reason={}".format(
                        decision["role"],
                        robot_name(decision["front_id"]) if decision["front_id"] is not None else "none",
                        robot_name(decision["child_id"]) if decision["child_id"] is not None else "none",
                        stable_reports,
                        LINE_LOCAL_STABLE_REQUIRED,
                        reason,
                    )
                )

        if (
            final_decision is not None
            and stable_reports >= LINE_LOCAL_STABLE_REQUIRED
            and now - start >= LINE_LOCAL_DECISION_AFTER_SECONDS
        ):
            print(
                "LINE FINAL: LOCAL role={} front={} child={} ({})".format(
                    final_decision["role"],
                    robot_name(final_decision["front_id"]) if final_decision["front_id"] is not None else "none",
                    robot_name(final_decision["child_id"]) if final_decision["child_id"] is not None else "none",
                    final_reason,
                )
            )
            drain_link(link, SMART_DRAIN_SECONDS)
            return final_decision

        time.sleep(POLL_INTERVAL)

    if final_decision is not None:
        print(
            "LINE FINAL: LOCAL role={} front={} child={} (timeout fallback: {})".format(
                final_decision["role"],
                robot_name(final_decision["front_id"]) if final_decision["front_id"] is not None else "none",
                robot_name(final_decision["child_id"]) if final_decision["child_id"] is not None else "none",
                final_reason,
            )
        )
        drain_link(link, SMART_DRAIN_SECONDS)
        return final_decision

    raise RuntimeError(
        "Line local role detection timed out after {:.1f}s. No mission movement started.".format(
            SMART_ELECTION_TIMEOUT
        )
    )

def navigation_motion_mode(navigator):
    """Map the goal navigator's internal state to a radio motion mode.
    
    Turn-related states map to TURN_TO_WALL, AVOID maps to AVOID, and all
    remaining states map to SEEK.
    """
    if navigator.state in ("TURN_TO_WALL", "TURN_CLEAR", "TURN_GAP"):
        return LeaderMotionMode.TURN_TO_WALL

    if navigator.state == "AVOID":
        return LeaderMotionMode.AVOID

    return LeaderMotionMode.SEEK


def encode_leader_motion_sequence(
    motion_mode,
    formation_slow=False,
    formation_hold=False,
):
    """Pack motion mode, formation-slow, and formation-hold into sequence bits.
    
    These bit flags are carried in the LEADER_MOVING message's sequence field.
    """
    sequence = int(motion_mode) & LEADER_SEQUENCE_MODE_MASK

    if formation_slow:
        sequence |= LEADER_SEQUENCE_SLOW_FORMATION_FLAG

    if formation_hold:
        sequence |= LEADER_SEQUENCE_HOLD_FORMATION_FLAG

    return sequence


def decode_leader_motion_sequence(sequence):
    """Unpack a LEADER_MOVING sequence into mode, slow and hold flags.
    
    Unknown motion mode numbers are interpreted as SEEK.
    """
    value = int(sequence)
    formation_slow = bool(value & LEADER_SEQUENCE_SLOW_FORMATION_FLAG)
    formation_hold = bool(value & LEADER_SEQUENCE_HOLD_FORMATION_FLAG)
    motion_value = value & LEADER_SEQUENCE_MODE_MASK

    try:
        motion_mode = LeaderMotionMode(motion_value)
    except ValueError:
        motion_mode = LeaderMotionMode.SEEK

    return motion_mode, formation_slow, formation_hold


def follower_ids_with_status(follower_status, wanted_status):
    """Return sorted IDs whose stored follower status matches a given value."""
    return sorted(
        follower_id
        for follower_id, status in follower_status.items()
        if status == wanted_status
    )


def weak_follower_links(now, follower_ids, last_seen):
    """Return followers whose latest packet age exceeds GOOD_PACKET_GAP."""
    return sorted(
        follower_id
        for follower_id in follower_ids
        if now - last_seen[follower_id] > GOOD_PACKET_GAP
    )



def follower_status_from_controller(controller, comm_state=None):
    """Convert local controller and communication states to an R&B status.
    
    LOCAL_YIELD means HOLDING; other LOCAL_* states mean AVOIDING; stale,
    lost, capture, or hold conditions mean HOLDING; otherwise NORMAL.
    """
    local_state = str(controller.local_state)

    # Yielding to the leader is not active obstacle avoidance.
    # The leader must be allowed to move away, otherwise both
    # robots can wait for each other indefinitely.
    if local_state == "LOCAL_YIELD":
        return FOLLOWER_STATUS_HOLDING

    # During actual obstacle avoidance, the leader and the
    # rest of the formation must stop.
    if local_state.startswith("LOCAL_"):
        return FOLLOWER_STATUS_AVOIDING

    if comm_state in ("STALE", "HOLD", "LOST", "CAPTURE"):
        return FOLLOWER_STATUS_HOLDING

    if local_state in ("HOLD", "CAPTURE"):
        return FOLLOWER_STATUS_HOLDING

    return FOLLOWER_STATUS_NORMAL



def follower_or_child_status(controller, comm_state=None, child_status=None):
    """Combine local and child statuses with AVOIDING taking precedence.
    
    This aggregation helper currently has no caller within this module.
    """
    own_status = follower_status_from_controller(controller, comm_state)

    if own_status == FOLLOWER_STATUS_AVOIDING:
        return FOLLOWER_STATUS_AVOIDING

    if child_status == FOLLOWER_STATUS_AVOIDING:
        return FOLLOWER_STATUS_AVOIDING

    if own_status == FOLLOWER_STATUS_HOLDING:
        return FOLLOWER_STATUS_HOLDING

    if child_status == FOLLOWER_STATUS_HOLDING:
        return FOLLOWER_STATUS_HOLDING

    return FOLLOWER_STATUS_NORMAL


def controller_is_locally_avoiding(controller):
    """Report controller LOCAL_* states except LOCAL_YIELD as active avoidance."""
    state = str(controller.local_state)
    return state.startswith("LOCAL_") and state != "LOCAL_YIELD"


def recovery_obstacle_active(recovery_navigator):
    """Read the recovery navigator's obstacle flag if one exists."""
    return (recovery_navigator is not None
            and recovery_navigator.obstacle_active)


def wifi_avoiding(wifi, robot_id):
    """Return a peer's obstacle flag only if its Wi-Fi report is <=1 s old.
    
    Fresh Wi-Fi obstacle reports supplement R&B status messages.
    """
    state = wifi.get_fresh_state(robot_id, max_age=1.0)
    return bool(state is not None and state.get('obstacle_avoiding') is True)


def wifi_recovery_target(wifi, front_id):
    """Get the front robot's fresh Wi-Fi displacement as an (x, y) target.
    
    The reported coordinates are local start-relative odometry values. Using
    them as the follower's target requires aligned initial odometry headings;
    no physical-origin correction or R&B-range scaling is performed.
    Return None if no report newer than 1.2 seconds is available.
    """
    pose = wifi.get_fresh_state(front_id, max_age=1.2)
    if pose is None:
        return None
    return (pose["x"], pose["y"])


def run_leader(
    puck,
    robot_id,
    follower_ids,
    link,
    slots,
    target_x,
    target_y,
):
    """Execute the goal-seeking leader role in a hub formation.
    
    Wait for every follower's READY packet; run ReachGoalController toward the
    requested millimeter goal; broadcast motion/formation flags through R&B;
    consume R&B heartbeats and fresh Wi-Fi obstacle reports; hold or slow based
    on follower state. Stop on SEARCH, prolonged lost contact, a stop signal,
    or completed navigation, and broadcast LEADER_STOPPED before returning.
    """
    ready_ids = set()
    last_seen = {
        follower_id: time.monotonic()
        for follower_id in follower_ids
    }
    last_status = 0.0
    follower_status = {
        follower_id: FOLLOWER_STATUS_NORMAL
        for follower_id in follower_ids
    }

    print("ROLE=LEADER LED=RED")
    print(
        "MISSION ROLE: {} is LEADER; followers={}".format(
            robot_name(robot_id),
            robot_list(follower_ids),
        )
    )
    print("Waiting for follower READY messages over R&B.")
    print("Mission R&B topology: LEADER_ONLY_HUB; follower-to-follower packets are ignored AFTER election only.")

    while (
        not _stop_requested
        and ready_ids != set(follower_ids)
    ):
        now = time.monotonic()

        link.send_in_slots(
            MessageType.LEADER_WAITING,
            slots,
        )

        reading, message = read_mission_packet(
            link,
            allowed_sender_ids=follower_ids,
            allowed_message_types=(
                MessageType.READY,
                MessageType.HEARTBEAT,
                MessageType.SEARCH,
            ),
        )

        if (
            reading is not None
            and message is not None
            and int(message.sender_id) in follower_ids
        ):
            follower_id = int(message.sender_id)
            last_seen[follower_id] = now

            if message.message_type == MessageType.READY:
                if follower_id not in ready_ids:
                    ready_ids.add(follower_id)
                    print(
                        "READY received from follower {}.".format(
                            follower_id
                        )
                    )

        if now - last_status >= STATUS_INTERVAL:
            print_mission_status(
                "self={} role=LEADER leader={} phase=WAITING_FOR_FOLLOWERS "
                "ready_followers={} missing_followers={} connections=({}) "
                "goal=NOT_STARTED rab_errors={} crc_errors={}".format(
                    robot_name(robot_id),
                    robot_name(robot_id),
                    robot_list(ready_ids),
                    robot_list(set(follower_ids) - ready_ids),
                    format_leader_connections(now, follower_ids, last_seen, follower_status),
                    link.i2c_errors,
                    link.crc_errors,
                )
            )
            last_status = now

        time.sleep(POLL_INTERVAL)

    if _stop_requested:
        return

    print("All followers READY. Starting slow stable reach_goal.")
    print_mission_status(
        "self={} role=LEADER leader={} phase=STARTING_MOVE "
        "ready_followers={} goal=RUNNING target=({:.1f},{:.1f})".format(
            robot_name(robot_id),
            robot_name(robot_id),
            robot_list(ready_ids),
            float(target_x),
            float(target_y),
        )
    )

    navigator = ReachGoalController(
        puck,
        target_x=target_x,
        target_y=target_y,
    )

    wifi = WorldStateNetwork(robot_id, "LEADER", navigator.odom)
    wifi.start()

    last_robot_update = 0.0
    last_status = 0.0

    while not _stop_requested and not navigator.finished:
        now = time.monotonic()
        motion_mode = navigation_motion_mode(navigator)

        pre_avoiding_ids = sorted(set(follower_ids_with_status(
            follower_status, FOLLOWER_STATUS_AVOIDING,
        )) | {f for f in follower_ids if wifi_avoiding(wifi, f)})
        pre_holding_ids = follower_ids_with_status(
            follower_status,
            FOLLOWER_STATUS_HOLDING,
        )
        pre_weak_link_ids = weak_follower_links(now, follower_ids, last_seen)
        formation_slow = bool(
            pre_avoiding_ids
            or pre_holding_ids
            or pre_weak_link_ids
        )
        formation_hold = bool(
            FORMATION_HOLD_ON_AVOIDING
            and pre_avoiding_ids
        )

        link.send_in_slots(
            MessageType.LEADER_MOVING,
            slots,
            sequence_override=encode_leader_motion_sequence(
                motion_mode,
                formation_slow,
                formation_hold,
            ),
        )

        reading, message = read_mission_packet(
            link,
            allowed_sender_ids=follower_ids,
            allowed_message_types=(
                MessageType.READY,
                MessageType.HEARTBEAT,
                MessageType.SEARCH,
            ),
        )

        if (
            reading is not None
            and message is not None
            and int(message.sender_id) in follower_ids
        ):
            follower_id = int(message.sender_id)
            last_seen[follower_id] = now

            if message.message_type == MessageType.HEARTBEAT:
                old = follower_status[follower_id]
                new = int(message.sequence)
                follower_status[follower_id] = new

                if new != old:
                    print(
                        "Follower {} status changed {} -> {}.".format(
                            follower_id,
                            old,
                            new,
                        )
                    )

            elif message.message_type == MessageType.SEARCH:
                print(
                    "Follower {} requested TEAM STOP.".format(
                        follower_id
                    )
                )
                navigator.stop()
                break

        lost = [
            follower_id
            for follower_id in follower_ids
            if now - last_seen[follower_id] > CONTACT_TIMEOUT
        ]

        if lost:
            print_mission_status(
                "self={} role=LEADER leader={} phase=TEAM_STOP "
                "connections=({}) goal=STOPPED reason=lost_followers lost_followers={}".format(
                    robot_name(robot_id),
                    robot_name(robot_id),
                    format_leader_connections(now, follower_ids, last_seen, follower_status),
                    robot_list(lost),
                )
            )
            print(
                "Lost R&B contact with followers {} for >{:.1f}s. TEAM STOP.".format(
                    lost,
                    CONTACT_TIMEOUT,
                )
            )
            navigator.stop()
            break

        avoiding_ids = sorted(set(follower_ids_with_status(
            follower_status, FOLLOWER_STATUS_AVOIDING,
        )) | {f for f in follower_ids if wifi_avoiding(wifi, f)})
        holding_ids = follower_ids_with_status(
            follower_status,
            FOLLOWER_STATUS_HOLDING,
        )
        weak_link_ids = weak_follower_links(now, follower_ids, last_seen)
        formation_slow = bool(
            avoiding_ids
            or holding_ids
            or weak_link_ids
        )
        formation_hold = bool(
            FORMATION_HOLD_ON_AVOIDING
            and avoiding_ids
        )

        if formation_hold:
            navigator.speed = 0
        elif holding_ids or weak_link_ids:
            navigator.speed = LEADER_FOLLOWER_HOLD_SPEED
        else:
            navigator.speed = LEADER_NORMAL_SPEED

        if now - last_robot_update >= ROBOT_UPDATE_PERIOD:
            frame = puck.epuck2.update()
            if formation_hold:
                navigator.stop()
            else:
                navigator.step(frame)
            last_robot_update = now

        if now - last_status >= STATUS_INTERVAL:
            x, y, theta = navigator.last_position
            print_mission_status(
                "self={} role=LEADER leader={} phase=MOVING nav={} mode={} speed={} formation_slow={} formation_hold={} "
                "connections=({}) avoiding_followers={} holding_followers={} weak_link_followers={} "
                "goal=RUNNING pos=({:.1f},{:.1f},{:.1f}) target=({:.1f},{:.1f}) "
                "rab_errors={} crc_errors={}".format(
                    robot_name(robot_id),
                    robot_name(robot_id),
                    navigator.state,
                    motion_mode.name,
                    navigator.speed,
                    str(formation_slow).lower(),
                    str(formation_hold).lower(),
                    format_leader_connections(now, follower_ids, last_seen, follower_status),
                    robot_list(avoiding_ids),
                    robot_list(holding_ids),
                    robot_list(weak_link_ids),
                    x,
                    y,
                    theta,
                    float(target_x),
                    float(target_y),
                    link.i2c_errors,
                    link.crc_errors,
                )
            )
            last_status = now

        time.sleep(POLL_INTERVAL)

    navigator.stop()
    set_all_rgb(puck, 0, 0, 0)

    stop_started = time.monotonic()

    while time.monotonic() - stop_started < 2.0:
        link.send_in_slots(
            MessageType.LEADER_STOPPED,
            slots,
        )
        link.read()
        time.sleep(POLL_INTERVAL)

    puck.epuck2.update()
    if navigator.finished:
        x, y, theta = navigator.last_position
        print_mission_status(
            "self={} role=LEADER leader={} phase=FINISHED goal=REACHED "
            "pos=({:.1f},{:.1f},{:.1f}) target=({:.1f},{:.1f})".format(
                robot_name(robot_id),
                robot_name(robot_id),
                x,
                y,
                theta,
                float(target_x),
                float(target_y),
            )
        )
    else:
        print_mission_status(
            "self={} role=LEADER leader={} phase=STOPPED goal=STOPPED".format(
                robot_name(robot_id),
                robot_name(robot_id),
            )
        )
    print("Leader mission stopped.")


def run_follower(
    puck,
    robot_id,
    leader_id,
    link,
    slots,
):
    """Execute the hub follower's R&B tracking and recovery state machine.
    
    CAPTURE stores the initial relative bearing/range; READY waits for leader
    movement; FOLLOW uses filtered R&B (or a recent stale snapshot), local
    proximity avoidance, and leader hold/slow flags. After prolonged R&B loss,
    WIFI_RECOVERY follows a fresh leader displacement, or BLIND_RECOVERY uses
    bounded forward motion; WAIT_RAB holds until usable information returns.
    The robot reports obstacle status via R&B and Wi-Fi. Stop messages terminate
    this role. Wheel control is delegated to imported controllers.
    """
    controller = RangeBearingFollower(puck)

    wifi = WorldStateNetwork(robot_id, "FOLLOWER", controller.odom)
    wifi.start()
    recovery_navigator = None
    recovery_started_at = None  # Monotonic timestamp for bounded recovery re-entry

    tracker = RangeAndBearingFilter(
        window_size=11,
        min_samples=5,
        stale_after=CONTACT_TIMEOUT,
        weak_threshold=200,
        strong_threshold=500,
    )

    state = "CAPTURE"

    last_leader = time.monotonic()
    last_good_snapshot = None
    last_good_snapshot_time = None

    last_robot_update = 0.0
    last_status = 0.0
    latest_frame = None
    current_comm_state = "CAPTURE"
    leader_motion_mode = LeaderMotionMode.SEEK
    leader_formation_slow = False
    leader_formation_hold = False

    print("ROLE=FOLLOWER LED=BLUE")
    print(
        "MISSION ROLE: {} is FOLLOWER; leader={}".format(
            robot_name(robot_id),
            robot_name(leader_id),
        )
    )
    print("Capturing the manually placed formation.")
    print(
        "Stable timing: GOOD<={:.1f}s STALE<={:.1f}s HOLD<={:.1f}s LOST>{:.1f}s".format(
            GOOD_PACKET_GAP,
            STALE_PACKET_GAP,
            LOST_PACKET_GAP,
            LOST_PACKET_GAP,
        )
    )
    print("Mission R&B topology: LEADER_ONLY_HUB; this follower ignores other followers AFTER election only.")

    while not _stop_requested:
        now = time.monotonic()
        frame_is_new = False

        if now - last_robot_update >= ROBOT_UPDATE_PERIOD:
            latest_frame = puck.epuck2.update()
            frame_is_new = True
            last_robot_update = now

        if state == "CAPTURE":
            outgoing = MessageType.HEARTBEAT
        elif state == "READY":
            outgoing = MessageType.READY
        elif state == "FOLLOW":
            outgoing = MessageType.HEARTBEAT
        elif state in ("BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB"):
            outgoing = MessageType.HEARTBEAT
        else:
            outgoing = MessageType.SEARCH

        if state in ("FOLLOW", "BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB"):
            pre_send_packet_age = now - last_leader
            if pre_send_packet_age > LOST_PACKET_GAP:
                current_comm_state = "LOST"
            elif pre_send_packet_age > STALE_PACKET_GAP:
                current_comm_state = "HOLD"
            elif pre_send_packet_age > GOOD_PACKET_GAP:
                current_comm_state = "STALE"

        sequence_override = None
        if outgoing == MessageType.HEARTBEAT:
            if state in ("BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB") and recovery_obstacle_active(recovery_navigator):
                sequence_override = FOLLOWER_STATUS_AVOIDING
            elif state in ("BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB"):
                sequence_override = FOLLOWER_STATUS_HOLDING
            else:
                sequence_override = follower_status_from_controller(
                    controller,
                    current_comm_state,
                )

        link.send_in_slots(
            outgoing,
            slots,
            sequence_override=sequence_override,
        )

        reading, message = read_mission_packet(
            link,
            allowed_sender_ids=(leader_id,),
            allowed_message_types=(
                MessageType.LEADER_WAITING,
                MessageType.LEADER_MOVING,
                MessageType.LEADER_STOPPED,
            ),
        )

        if (
            reading is not None
            and message is not None
            and int(message.sender_id) == leader_id
        ):
            last_leader = now

            if message.message_type in (
                MessageType.LEADER_WAITING,
                MessageType.LEADER_MOVING,
                MessageType.LEADER_STOPPED,
            ):
                tracker.add(reading, now=now)

            if message.message_type == MessageType.LEADER_MOVING:
                leader_motion_mode, leader_formation_slow, leader_formation_hold = decode_leader_motion_sequence(
                    message.sequence
                )

                if state == "READY":
                    print("Leader started. Entering FOLLOW.")
                    print_mission_status(
                        "self={} role=FOLLOWER leader={} phase=FOLLOW "
                        "connection_to_leader=OK goal=RUNNING".format(
                            robot_name(robot_id),
                            robot_name(leader_id),
                        )
                    )
                    state = "FOLLOW"
                    current_comm_state = "GOOD"
                elif state == "FOLLOW":
                    pass

            elif message.message_type == MessageType.LEADER_STOPPED:
                print("Leader stopped. Follower stopping.")
                print_mission_status(
                    "self={} role=FOLLOWER leader={} phase=FINISHED "
                    "connection_to_leader=OK goal=LEADER_STOPPED".format(
                        robot_name(robot_id),
                        robot_name(leader_id),
                    )
                )
                controller.stop()
                puck.epuck2.update()
                break

        snapshot = tracker.snapshot(now=now)

        if snapshot is not None:
            last_good_snapshot = snapshot
            last_good_snapshot_time = now

        if state == "CAPTURE":
            if controller.capture_step(snapshot):
                state = "READY"
                current_comm_state = "READY"
                print(
                    "Captured live formation: bearing={:+.2f}, "
                    "range={:.1f}, peak={:.1f}. "
                    "Sending READY over R&B.".format(
                        controller.target_bearing,
                        controller.target_range,
                        controller.target_peak,
                    )
                )
                print_mission_status(
                    "self={} role=FOLLOWER leader={} phase=READY "
                    "connection_to_leader=OK goal=WAITING_FOR_LEADER "
                    "target_bearing={:+.2f} target_range={:.1f}".format(
                        robot_name(robot_id),
                        robot_name(leader_id),
                        controller.target_bearing,
                        controller.target_range,
                    )
                )

        elif state == "READY":
            controller.stop()

        elif state == "FOLLOW":
            packet_age = now - last_leader
            good_age = None
            if last_good_snapshot_time is not None:
                good_age = now - last_good_snapshot_time

            leader_is_turning = leader_motion_mode != LeaderMotionMode.SEEK
            normal_speed_scale = 0.45 if leader_is_turning else 1.0
            stale_speed_scale = 0.60 if leader_is_turning else 0.80

            if leader_formation_slow:
                normal_speed_scale = min(
                    normal_speed_scale,
                    FOLLOWER_FORMATION_SLOW_SCALE,
                )
                stale_speed_scale = min(
                    stale_speed_scale,
                    FOLLOWER_FORMATION_SLOW_STALE_SCALE,
                )

            local_chain_hold = leader_formation_hold or leader_is_turning

            if packet_age > LOST_PACKET_GAP:
                current_comm_state = "LOST"
                # On lost R&B, use fresh Wi-Fi displacement when available;
                # otherwise begin bounded blind recovery. No R&B-mm conversion.
                recovery_started_at = now
                recovery_navigator = WiFiDisplacementRecovery(puck, duration_seconds=10.0)
                target = wifi_recovery_target(wifi, leader_id)
                state = "WIFI_RECOVERY" if target is not None else "BLIND_RECOVERY"
                print("R&B lost: Wi-Fi displacement target when fresh, otherwise bounded straight recovery.", flush=True)
                if frame_is_new and latest_frame is not None:
                    # Integrate each encoder frame once; the recovery navigator does not.
                    controller.odom.update(*latest_frame.motor_steps)
                    recovery_navigator.step(latest_frame, math.degrees(controller.odom.theta),
                                            (controller.odom.x, controller.odom.y), wifi_target=target)
                if not frame_is_new:
                    controller.stop()
                status = None

            elif local_chain_hold and not controller_is_locally_avoiding(controller):
                current_comm_state = "FORMATION_HOLD"
                status = controller.hold(frame=latest_frame)

            elif snapshot is not None and packet_age <= GOOD_PACKET_GAP:
                current_comm_state = "GOOD"
                
                # Keep odometry as displacement from this robot's own start.
                # Wi-Fi waypoint recovery compares aligned START-FRAME displacements,
                # so blending a peer's local pose and raw R&B distance here would
                # silently corrupt the coordinate frame.
                status = controller.follow_step(
                    snapshot,
                    frame=latest_frame,
                    speed_scale=normal_speed_scale,
                    wifi_state=wifi.teammates_state,
                    my_id=robot_id,
                    leader_id=leader_id
                )

            elif (
                last_good_snapshot is not None
                and good_age is not None
                and good_age <= STALE_PACKET_GAP
            ):
                current_comm_state = "STALE"
                status = controller.follow_step(
                    last_good_snapshot,
                    frame=latest_frame,
                    speed_scale=stale_speed_scale,
                    wifi_state=wifi.teammates_state,
                    my_id=robot_id,
                    leader_id=leader_id
                )

            else:
                current_comm_state = "HOLD"
                status = controller.hold(frame=latest_frame)

        elif state in ("BLIND_RECOVERY", "WIFI_RECOVERY"):
            packet_age = now - last_leader
            if snapshot is not None and packet_age <= GOOD_PACKET_GAP:
                print("R&B reacquired: returning to normal FOLLOW.", flush=True)
                recovery_navigator.stop()
                state = "FOLLOW"
                recovery_started_at = None
                current_comm_state = "GOOD"
                # R&B takes priority; restore the regular proximity-aware follower.
                controller.local_state = "FOLLOW"
                if frame_is_new:
                    status = controller.follow_step(
                        snapshot, frame=latest_frame, speed_scale=1.0,
                        wifi_state=wifi.teammates_state,
                        my_id=robot_id, leader_id=leader_id,
                    )
                else:
                    status = None
            else:
                current_comm_state = "LOST"
                target = wifi_recovery_target(wifi, leader_id)
                if frame_is_new and latest_frame is not None:
                    controller.odom.update(*latest_frame.motor_steps)
                    recovery_navigator.step(latest_frame, math.degrees(controller.odom.theta),
                                            (controller.odom.x, controller.odom.y), wifi_target=target)
                if recovery_navigator.done:
                    # Blind travel is bounded; a fresh Wi-Fi target can still
                    # reactivate waypoint recovery from WAIT_RAB.
                    state = "WAIT_RAB"
                    print("Recovery ended ({}): holding for R&B or Wi-Fi target.".format(
                        recovery_navigator.reason), flush=True)
                else:
                    state = "WIFI_RECOVERY" if target is not None else "BLIND_RECOVERY"
                status = {
                    "local_state": "RECOVERY_" + recovery_navigator.state,
                    "left": recovery_navigator.left,
                    "right": recovery_navigator.right,
                    "bearing": None, "range": None, "target_range": None,
                    "range_state": "RECOVERY",
                }

        elif state == "WAIT_RAB":
            packet_age = now - last_leader
            if snapshot is not None and packet_age <= GOOD_PACKET_GAP:
                print("R&B reacquired: resuming original FOLLOW controller.", flush=True)
                state = "FOLLOW"
                recovery_started_at = None
                current_comm_state = "GOOD"
                # R&B takes priority; restore the regular proximity-aware follower.
                controller.local_state = "FOLLOW"
                if frame_is_new:
                    status = controller.follow_step(
                        snapshot, frame=latest_frame, speed_scale=1.0,
                        wifi_state=wifi.teammates_state,
                        my_id=robot_id, leader_id=leader_id,
                    )
                else:
                    status = None
            else:
                current_comm_state = "LOST"
                target = wifi_recovery_target(wifi, leader_id)
                if (target is not None and recovery_started_at is not None
                        and now - recovery_started_at < 60.0):
                    recovery_navigator = WiFiDisplacementRecovery(puck, duration_seconds=10.0)
                    # Do not grant a new recovery window after an earlier timeout.
                    recovery_navigator.started = recovery_started_at
                    state = "WIFI_RECOVERY"
                    print("Fresh Wi-Fi displacement target acquired; leaving WAIT_RAB.", flush=True)
                    if frame_is_new and latest_frame is not None:
                        controller.odom.update(*latest_frame.motor_steps)
                        recovery_navigator.step(latest_frame, math.degrees(controller.odom.theta),
                                                (controller.odom.x, controller.odom.y), wifi_target=target)
                    status = None
                else:
                    status = controller.hold(frame=latest_frame if frame_is_new else None)

        else:
            controller.stop()

        wifi.set_obstacle_avoiding(
            (state == "FOLLOW" and controller_is_locally_avoiding(controller))
            or (state in ("BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB") and recovery_obstacle_active(recovery_navigator))
        )

        if now - last_status >= STATUS_INTERVAL:
            if state in ("FOLLOW", "BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB"):
                if status is None:
                    print_mission_status(
                        "self={} role=FOLLOWER leader={} phase=FOLLOW "
                        "connection_to_leader={} age={:.1f}s leader_mode={} formation_slow={} formation_hold={} local=HOLD goal=RUNNING "
                        "rab_errors={} crc_errors={}".format(
                            robot_name(robot_id),
                            robot_name(leader_id),
                            current_comm_state,
                            packet_age,
                            leader_motion_mode.name,
                            str(leader_formation_slow).lower(),
                            str(leader_formation_hold).lower(),
                            link.i2c_errors,
                            link.crc_errors,
                        )
                    )
                else:
                    print_mission_status(
                        "self={} role=FOLLOWER leader={} phase=FOLLOW "
                        "connection_to_leader={} age={:.1f}s leader_mode={} formation_slow={} formation_hold={} local={} goal=RUNNING "
                        "bearing={} range={} target_range={} range_state={} wheels=({}, {}) "
                        "rab_errors={} crc_errors={}".format(
                            robot_name(robot_id),
                            robot_name(leader_id),
                            current_comm_state,
                            packet_age,
                            leader_motion_mode.name,
                            str(leader_formation_slow).lower(),
                            str(leader_formation_hold).lower(),
                            status.get("local_state"),
                            _fmt(status.get("bearing")),
                            _fmt(status.get("range")),
                            _fmt(status.get("target_range")),
                            status.get("range_state", "-"),
                            int(status.get("left", 0)),
                            int(status.get("right", 0)),
                            link.i2c_errors,
                            link.crc_errors,
                        )
                    )
            elif state == "CAPTURE":
                print_mission_status("self={} role=FOLLOWER phase=CAPTURE goal=WAITING_TO_SEE_LEADER".format(robot_name(robot_id)))
            elif state == "READY":
                print_mission_status("self={} role=FOLLOWER phase=READY goal=WAITING_FOR_LEADER_START".format(robot_name(robot_id)))
            
            last_status = now

        time.sleep(POLL_INTERVAL)

    controller.stop()
    puck.epuck2.update()



def run_line_leader(
    puck,
    robot_id,
    child_id,
    link,
    slots,
    target_x,
    target_y,
):
    """Execute the front leader in a line formation.
    
    Wait for the direct child, navigate toward the goal, and transmit motion
    and hold flags downstream. Interpret child status from R&B and fresh Wi-Fi
    avoidance; pause for a child's obstacle while allowing the leader's own
    obstacle-clearing maneuver to proceed. Propagate the final STOPPED message.
    """
    if child_id is None:
        run_leader(puck, robot_id, [], link, slots, target_x, target_y)
        return

    ready_ids = set()
    last_seen = {int(child_id): time.monotonic()}
    follower_status = {int(child_id): FOLLOWER_STATUS_NORMAL}
    last_status = 0.0

    print("ROLE=LINE_LEADER LED=RED")
    print(
        "MISSION ROLE: {} is LINE_LEADER; child={}".format(
            robot_name(robot_id),
            robot_name(child_id),
        )
    )
    print("Line leader waits only for its direct child READY message.")

    while not _stop_requested and int(child_id) not in ready_ids:
        now = time.monotonic()
        link.send_in_slots(MessageType.LEADER_WAITING, slots)

        reading, message = read_mission_packet(
            link,
            allowed_sender_ids=[int(child_id)],
            allowed_message_types=(
                MessageType.READY,
                MessageType.HEARTBEAT,
                MessageType.SEARCH,
                MessageType.LEADER_MOVING,
            ),
            max_reads=10,
        )
        
        if (
            reading is not None
            and message is not None
            and int(message.sender_id) == int(child_id)
        ):
            last_seen[int(child_id)] = now
            if message.message_type == MessageType.READY:
                ready_ids.add(int(child_id))
                print("READY received from line child {}.".format(child_id))

        if now - last_status >= STATUS_INTERVAL:
            print_mission_status(
                "self={} role=LINE_LEADER phase=WAITING_FOR_CHILD child={} child_ready={} "
                "connection=({}) goal=NOT_STARTED rab_errors={} crc_errors={}".format(
                    robot_name(robot_id),
                    robot_name(child_id),
                    str(int(child_id) in ready_ids).lower(),
                    format_leader_connections(now, [int(child_id)], last_seen, follower_status),
                    link.i2c_errors,
                    link.crc_errors,
                )
            )
            last_status = now

        time.sleep(POLL_INTERVAL)

    if _stop_requested:
        return

    print("Direct child READY. Line leader starting reach_goal.")
    print_mission_status(
        "self={} role=LINE_LEADER phase=STARTING_MOVE child={} goal=RUNNING target=({:.1f},{:.1f})".format(
            robot_name(robot_id),
            robot_name(child_id),
            float(target_x),
            float(target_y),
        )
    )

    navigator = ReachGoalController(puck, target_x=target_x, target_y=target_y)

    wifi = WorldStateNetwork(robot_id, "LINE_LEADER", navigator.odom)
    wifi.start()

    last_robot_update = 0.0
    last_status = 0.0

    while not _stop_requested and not navigator.finished:
        now = time.monotonic()
        motion_mode = navigation_motion_mode(navigator)

        child_status = follower_status.get(int(child_id), FOLLOWER_STATUS_NORMAL)
        child_age = now - last_seen[int(child_id)]
        is_avoiding = motion_mode in (LeaderMotionMode.TURN_TO_WALL, LeaderMotionMode.AVOID)

        # A stale R&B AVOIDING bit must not lock the line forever. Fresh Wi-Fi
        # can still carry the avoidance flag while R&B is obstructed.
        child_avoiding = (
            (child_age <= LOST_PACKET_GAP and child_status == FOLLOWER_STATUS_AVOIDING)
            or wifi_avoiding(wifi, int(child_id))
        )
        child_holding = (child_age <= LOST_PACKET_GAP
                         and child_status == FOLLOWER_STATUS_HOLDING)
        # Advertise a hold during our own obstacle turn so the middle waits;
        # or during a child's obstacle maneuver so the whole chain waits.
        formation_hold = is_avoiding or child_avoiding
        formation_slow = child_holding  # Do not slow the child just because uplink R&B is weak

        link.send_in_slots(
            MessageType.LEADER_MOVING,
            slots,
            sequence_override=encode_leader_motion_sequence(
                motion_mode,
                formation_slow,
                formation_hold,
            ),
        )

        reading, message = read_mission_packet(
            link,
            allowed_sender_ids=[int(child_id)],
            allowed_message_types=(
                MessageType.READY,
                MessageType.HEARTBEAT,
                MessageType.SEARCH,
                MessageType.LEADER_MOVING,
            ),
            max_reads=10,
        )
        if (
            reading is not None
            and message is not None
            and int(message.sender_id) == int(child_id)
        ):
            last_seen[int(child_id)] = now
            if message.message_type == MessageType.SEARCH:
                print("Line child {} requested TEAM STOP.".format(child_id))
                navigator.stop()
                break
            elif message.message_type == MessageType.HEARTBEAT:
                follower_status[int(child_id)] = int(message.sequence)
            elif message.message_type == MessageType.LEADER_MOVING:
                _, c_slow, c_hold = decode_leader_motion_sequence(message.sequence)
                if c_slow:
                    follower_status[int(child_id)] = FOLLOWER_STATUS_AVOIDING
                elif c_hold:
                    follower_status[int(child_id)] = FOLLOWER_STATUS_HOLDING
                else:
                    follower_status[int(child_id)] = FOLLOWER_STATUS_NORMAL

        if now - last_seen[int(child_id)] > CONTACT_TIMEOUT:
            print_mission_status(
                "self={} role=LINE_LEADER phase=TEAM_STOP child={} goal=STOPPED reason=lost_child connection=({})".format(
                    robot_name(robot_id),
                    robot_name(child_id),
                    format_leader_connections(now, [int(child_id)], last_seen, follower_status),
                )
            )
            navigator.stop()
            break

        child_status = follower_status.get(int(child_id), FOLLOWER_STATUS_NORMAL)
        child_age = now - last_seen[int(child_id)]
        is_avoiding = motion_mode in (LeaderMotionMode.TURN_TO_WALL, LeaderMotionMode.AVOID)
        child_avoiding = (
            (child_age <= LOST_PACKET_GAP and child_status == FOLLOWER_STATUS_AVOIDING)
            or wifi_avoiding(wifi, int(child_id))
        )
        child_holding = (child_age <= LOST_PACKET_GAP
                         and child_status == FOLLOWER_STATUS_HOLDING)

        # If the child meets an obstacle, stop our normal goal navigation.
        # If WE are already clearing an obstacle, we must finish that turn.
        is_holding_for_child = child_avoiding and not is_avoiding
        if is_holding_for_child:
            navigator.speed = 0
        elif child_holding or child_age > GOOD_PACKET_GAP:
            navigator.speed = LEADER_FOLLOWER_HOLD_SPEED
        else:
            navigator.speed = LEADER_NORMAL_SPEED

        if now - last_robot_update >= ROBOT_UPDATE_PERIOD:
            frame = puck.epuck2.update()
            if is_holding_for_child:
                navigator.stop()
            else:
                navigator.step(frame)
            last_robot_update = now

        if now - last_status >= STATUS_INTERVAL:
            x, y, theta = navigator.last_position
            print_mission_status(
                "self={} role=LINE_LEADER phase=MOVING nav={} mode={} speed={} formation_slow={} formation_hold={} child={} "
                "connection=({}) goal=RUNNING pos=({:.1f},{:.1f},{:.1f}) target=({:.1f},{:.1f}) "
                "rab_errors={} crc_errors={}".format(
                    robot_name(robot_id),
                    navigator.state,
                    motion_mode.name,
                    navigator.speed,
                    str(formation_slow).lower(),
                    str(formation_hold).lower(),
                    robot_name(child_id),
                    format_leader_connections(now, [int(child_id)], last_seen, follower_status),
                    x,
                    y,
                    theta,
                    float(target_x),
                    float(target_y),
                    link.i2c_errors,
                    link.crc_errors,
                )
            )
            last_status = now

        time.sleep(POLL_INTERVAL)

    navigator.stop()
    set_all_rgb(puck, 0, 0, 0)

    stop_started = time.monotonic()
    while time.monotonic() - stop_started < 2.0:
        link.send_in_slots(MessageType.LEADER_STOPPED, slots)
        link.read()
        time.sleep(POLL_INTERVAL)

    puck.epuck2.update()
    if navigator.finished:
        x, y, theta = navigator.last_position
        print_mission_status(
            "self={} role=LINE_LEADER phase=FINISHED goal=REACHED pos=({:.1f},{:.1f},{:.1f}) target=({:.1f},{:.1f})".format(
                robot_name(robot_id),
                x,
                y,
                theta,
                float(target_x),
                float(target_y),
            )
        )
    else:
        print_mission_status(
            "self={} role=LINE_LEADER phase=STOPPED goal=STOPPED".format(
                robot_name(robot_id)
            )
        )
    print("Line leader mission stopped.")


def run_line_follower(
    puck,
    robot_id,
    front_id,
    child_id,
    link,
    slots,
):
    """Execute either middle/relay or tail follower behavior in a line.
    
    The robot follows only its direct front neighbor using captured R&B
    bearing/range. With a child, it additionally waits for child readiness,
    forwards motion and avoidance information, and relays stop requests.
    Local mission states include CAPTURE, READY, FOLLOW, BLIND_RECOVERY,
    WIFI_RECOVERY, WAIT_RAB, and SAFETY_STOP. Wi-Fi gives fresh displacement
    waypoints during R&B loss; proximity avoidance remains controller-owned.
    """
    controller = RangeBearingFollower(puck)


    tracker = RangeAndBearingFilter(
        window_size=11,
        min_samples=5,
        stale_after=CONTACT_TIMEOUT,
        weak_threshold=200,
        strong_threshold=500,
    )

    is_local_leader = child_id is not None
    role_name = "LOCAL_LEADER" if is_local_leader else "FOLLOWER"
    led = (60, 0, 100) if is_local_leader else (0, 0, 100)
    set_all_rgb(puck, *led)

    wifi = WorldStateNetwork(robot_id, role_name, controller.odom)
    wifi.start()
    recovery_navigator = None
    recovery_started_at = None  # Monotonic timestamp for bounded recovery re-entry

    state = "CAPTURE"
    child_ready = child_id is None
    last_front = time.monotonic()
    last_child = time.monotonic()
    last_good_snapshot = None
    last_good_snapshot_time = None
    latest_frame = None
    last_robot_update = 0.0
    last_status = 0.0
    current_comm_state = "CAPTURE"
    front_motion_mode = LeaderMotionMode.SEEK
    front_formation_slow = False
    front_formation_hold = False
    child_status = FOLLOWER_STATUS_NORMAL
    stop_requested_by_child = False
    follow_started_at = None

    print("ROLE={} LED={}".format(role_name, "PURPLE" if is_local_leader else "BLUE"))
    print(
        "MISSION ROLE: {} is {}; front={} child={}".format(
            robot_name(robot_id),
            role_name,
            robot_name(front_id),
            robot_name(child_id) if child_id is not None else "none",
        )
    )
    print("Line mode: this robot follows only its direct front robot.")

    while not _stop_requested:
        now = time.monotonic()
        frame_is_new = False

        if now - last_robot_update >= ROBOT_UPDATE_PERIOD:
            latest_frame = puck.epuck2.update()
            frame_is_new = True
            last_robot_update = now

        cycle, _slot = cycle_and_slot(time.time())

        # LOCAL_* includes obstacle turning and forward clearance, but not
        # gentle side steering; LOCAL_YIELD is explicitly excluded.
        is_locally_avoiding = controller_is_locally_avoiding(controller)
        child_avoiding = (child_id is not None and (
            (now - last_child <= LOST_PACKET_GAP and
             child_status == FOLLOWER_STATUS_AVOIDING)
            or wifi_avoiding(wifi, child_id)
        ))

        if stop_requested_by_child or state == "SAFETY_STOP":
            outgoing = MessageType.SEARCH
            sequence_override = 0
        elif state == "CAPTURE":
            if is_local_leader:
                outgoing = MessageType.LEADER_WAITING
            else:
                outgoing = MessageType.HEARTBEAT
            sequence_override = 0
        elif state == "READY":
            if is_local_leader:
                if child_ready:
                    outgoing = MessageType.READY
                else:
                    outgoing = MessageType.LEADER_WAITING
            else:
                outgoing = MessageType.READY
            sequence_override = 0
        elif state in ("FOLLOW", "BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB"):
            if is_local_leader:
                outgoing = MessageType.LEADER_MOVING
                
                # Relay REAL obstacle avoidance upstream using the AVOID bit.
                # Being a local leader does not change our own driving PID.
                my_slow = (is_locally_avoiding
                           or (state in ("BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB")
                               and recovery_obstacle_active(recovery_navigator))
                           or child_avoiding)

                # Downstream robots hold during another robot's obstacle turn.
                # A pure R&B dropout is not, by itself, an obstacle.
                my_hold = (front_formation_hold or is_locally_avoiding
                           or (state in ("BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB")
                               and recovery_obstacle_active(recovery_navigator))
                           or child_avoiding)
                
                sequence_override = encode_leader_motion_sequence(
                    front_motion_mode,
                    my_slow,
                    my_hold,
                )
            else:
                outgoing = MessageType.HEARTBEAT
                if (state in ("BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB") and recovery_obstacle_active(recovery_navigator)):
                    sequence_override = FOLLOWER_STATUS_AVOIDING
                elif state in ("BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB"):
                    sequence_override = FOLLOWER_STATUS_HOLDING
                elif is_locally_avoiding or child_avoiding:
                    sequence_override = FOLLOWER_STATUS_AVOIDING
                elif front_formation_hold:
                    sequence_override = FOLLOWER_STATUS_HOLDING
                else:
                    sequence_override = FOLLOWER_STATUS_NORMAL
        else:
            outgoing = MessageType.SEARCH
            sequence_override = 0

        link.send_in_slots(
            outgoing,
            slots,
            sequence_override=sequence_override,
        )

        allowed_senders = [int(front_id)]
        if child_id is not None:
            allowed_senders.append(int(child_id))

        reading, message = read_mission_packet(
            link,
            allowed_sender_ids=allowed_senders,
            allowed_message_types={
                MessageType.LEADER_WAITING,
                MessageType.LEADER_MOVING,
                MessageType.LEADER_STOPPED,
                MessageType.READY,
                MessageType.HEARTBEAT,
                MessageType.SEARCH,
            },
            max_reads=10,
        )
        if reading is not None and message is not None:
            sender_id = int(message.sender_id)

            if sender_id == int(front_id):
                last_front = now
                tracker.add(reading, now=now)

                if message.message_type == MessageType.LEADER_MOVING:
                    front_motion_mode, front_formation_slow, front_formation_hold = decode_leader_motion_sequence(message.sequence)
                    if state == "READY" and child_ready:
                        print("Front robot started. Entering FOLLOW.")
                        print_mission_status(
                            "self={} role={} front={} phase=FOLLOW child_ready={} connection_to_front=OK goal=RUNNING".format(
                                robot_name(robot_id),
                                role_name,
                                robot_name(front_id),
                                str(child_ready).lower(),
                            )
                        )
                        state = "FOLLOW"
                        current_comm_state = "GOOD"

                elif message.message_type == MessageType.LEADER_STOPPED:
                    print("Front robot stopped. Line follower stopping.")
                    print_mission_status(
                        "self={} role={} front={} phase=FINISHED connection_to_front=OK goal=FRONT_STOPPED".format(
                            robot_name(robot_id),
                            role_name,
                            robot_name(front_id),
                        )
                    )
                    controller.stop()
                    puck.epuck2.update()
                    if is_local_leader:
                        end = time.monotonic() + 2.0
                        while time.monotonic() < end:
                            link.send_in_slots(MessageType.LEADER_STOPPED, slots)
                            link.read()
                            time.sleep(POLL_INTERVAL)
                    break

            elif child_id is not None and sender_id == int(child_id):
                last_child = now
                if message.message_type == MessageType.READY and not child_ready:
                    child_ready = True
                    print("READY received from line child {}.".format(child_id))
                elif message.message_type == MessageType.HEARTBEAT:
                    old_status = child_status
                    child_status = int(message.sequence)
                    if child_status != old_status:
                        print("Line child {} status changed {} -> {}.".format(child_id, old_status, child_status))
                elif message.message_type == MessageType.LEADER_MOVING:
                    _, c_slow, c_hold = decode_leader_motion_sequence(message.sequence)
                    old_status = child_status
                    if c_slow:
                        child_status = FOLLOWER_STATUS_AVOIDING
                    elif c_hold:
                        child_status = FOLLOWER_STATUS_HOLDING
                    else:
                        child_status = FOLLOWER_STATUS_NORMAL
                    if child_status != old_status:
                        print("Line child {} status changed {} -> {}.".format(child_id, old_status, child_status))
                elif message.message_type == MessageType.SEARCH:
                    print("Line child {} requested TEAM STOP.".format(child_id))
                    stop_requested_by_child = True
                    state = "SAFETY_STOP"
                    controller.stop()

        snapshot = tracker.snapshot(now=now)
        if snapshot is not None:
            last_good_snapshot = snapshot
            last_good_snapshot_time = now

        if state == "CAPTURE":
            if controller.capture_step(snapshot):
                state = "READY"
                current_comm_state = "READY"
                print(
                    "Captured line front robot: front={} bearing={:+.2f}, range={:.1f}, peak={:.1f}.".format(
                        robot_name(front_id),
                        controller.target_bearing,
                        controller.target_range,
                        controller.target_peak,
                    )
                )
                print_mission_status(
                    "self={} role={} front={} child={} phase=READY child_ready={} "
                    "connection_to_front=OK goal=WAITING_FOR_FRONT target_bearing={:+.2f} target_range={:.1f}".format(
                        robot_name(robot_id),
                        role_name,
                        robot_name(front_id),
                        robot_name(child_id) if child_id is not None else "none",
                        str(child_ready).lower(),
                        controller.target_bearing,
                        controller.target_range,
                    )
                )

        elif state == "READY":
            controller.stop()

        elif state == "FOLLOW":
            packet_age = now - last_front
            good_age = None
            if last_good_snapshot_time is not None:
                good_age = now - last_good_snapshot_time

            front_is_turning = front_motion_mode != LeaderMotionMode.SEEK
            # Same tracking speed policy as hub followers. The middle robot
            # has no special cap merely because it relays a child's packets.
            normal_speed_scale = 0.45 if front_is_turning else 1.0
            stale_speed_scale = 0.60 if front_is_turning else 0.80

            if front_formation_slow:
                normal_speed_scale = min(
                    normal_speed_scale,
                    FOLLOWER_FORMATION_SLOW_SCALE,
                )
                stale_speed_scale = min(
                    stale_speed_scale,
                    FOLLOWER_FORMATION_SLOW_STALE_SCALE,
                )

            local_chain_hold = bool(front_formation_hold) or front_is_turning or child_avoiding

            # Do not drive blind during a known team obstacle hold. The
            # active obstacle-clearing robot remains free to pivot below.
            if packet_age > LOST_PACKET_GAP and not local_chain_hold:
                current_comm_state = "LOST"
                # On lost R&B, use fresh Wi-Fi displacement when available;
                # otherwise begin bounded blind recovery. No R&B-mm conversion.
                recovery_started_at = now
                recovery_navigator = WiFiDisplacementRecovery(puck, duration_seconds=10.0)
                target = wifi_recovery_target(wifi, front_id)
                state = "WIFI_RECOVERY" if target is not None else "BLIND_RECOVERY"
                print("R&B lost: Wi-Fi displacement target when fresh, otherwise bounded straight recovery.", flush=True)
                if frame_is_new and latest_frame is not None:
                    # Integrate each encoder frame once; the recovery navigator does not.
                    controller.odom.update(*latest_frame.motor_steps)
                    recovery_navigator.step(latest_frame, math.degrees(controller.odom.theta),
                                            (controller.odom.x, controller.odom.y), wifi_target=target)
                if not frame_is_new:
                    controller.stop()
                status = None

            elif local_chain_hold and not controller_is_locally_avoiding(controller):
                current_comm_state = "FORMATION_HOLD"
                status = controller.hold(frame=latest_frame)

            elif snapshot is not None and packet_age <= GOOD_PACKET_GAP:
                current_comm_state = "GOOD"
                
                # Preserve this follower's start-relative odometry coordinates.
                # Both Wi-Fi waypoint recovery and the front robot's transmitted
                # (x,y) use displacements from their respective initial origins.
                status = controller.follow_step(
                    snapshot,
                    frame=latest_frame,
                    speed_scale=normal_speed_scale,
                    wifi_state=wifi.teammates_state,
                    my_id=robot_id,
                    leader_id=front_id
                )
            elif (
                last_good_snapshot is not None
                and good_age is not None
                and good_age <= STALE_PACKET_GAP
            ):
                current_comm_state = "STALE"
                status = controller.follow_step(
                    last_good_snapshot,
                    frame=latest_frame,
                    speed_scale=stale_speed_scale,
                    wifi_state=wifi.teammates_state,
                    my_id=robot_id,
                    leader_id=front_id
                )
            elif packet_age <= LOST_PACKET_GAP:
                current_comm_state = "HOLD"
                status = controller.hold(frame=latest_frame)

        elif state in ("BLIND_RECOVERY", "WIFI_RECOVERY"):
            packet_age = now - last_front
            if snapshot is not None and packet_age <= GOOD_PACKET_GAP:
                print("R&B reacquired: returning to normal FOLLOW.", flush=True)
                recovery_navigator.stop()
                state = "FOLLOW"
                recovery_started_at = None
                current_comm_state = "GOOD"
                # R&B takes priority; restore the regular proximity-aware follower.
                controller.local_state = "FOLLOW"
                if frame_is_new:
                    status = controller.follow_step(
                        snapshot, frame=latest_frame, speed_scale=1.0,
                        wifi_state=wifi.teammates_state,
                        my_id=robot_id, leader_id=front_id,
                    )
                else:
                    status = None
            else:
                current_comm_state = "LOST"
                target = wifi_recovery_target(wifi, front_id)
                if frame_is_new and latest_frame is not None:
                    controller.odom.update(*latest_frame.motor_steps)
                    recovery_navigator.step(latest_frame, math.degrees(controller.odom.theta),
                                            (controller.odom.x, controller.odom.y), wifi_target=target)
                if recovery_navigator.done:
                    # Blind travel is bounded; a fresh Wi-Fi target can still
                    # reactivate waypoint recovery from WAIT_RAB.
                    state = "WAIT_RAB"
                    print("Recovery ended ({}): holding for R&B or Wi-Fi target.".format(
                        recovery_navigator.reason), flush=True)
                else:
                    state = "WIFI_RECOVERY" if target is not None else "BLIND_RECOVERY"
                status = {
                    "local_state": "RECOVERY_" + recovery_navigator.state,
                    "left": recovery_navigator.left,
                    "right": recovery_navigator.right,
                    "bearing": None, "range": None, "target_range": None,
                    "range_state": "RECOVERY",
                }

        elif state == "WAIT_RAB":
            packet_age = now - last_front
            if snapshot is not None and packet_age <= GOOD_PACKET_GAP:
                print("R&B reacquired: resuming original FOLLOW controller.", flush=True)
                state = "FOLLOW"
                recovery_started_at = None
                current_comm_state = "GOOD"
                # R&B takes priority; restore the regular proximity-aware follower.
                controller.local_state = "FOLLOW"
                if frame_is_new:
                    status = controller.follow_step(
                        snapshot, frame=latest_frame, speed_scale=1.0,
                        wifi_state=wifi.teammates_state,
                        my_id=robot_id, leader_id=front_id,
                    )
                else:
                    status = None
            else:
                current_comm_state = "LOST"
                target = wifi_recovery_target(wifi, front_id)
                if (target is not None and recovery_started_at is not None
                        and now - recovery_started_at < 60.0):
                    recovery_navigator = WiFiDisplacementRecovery(puck, duration_seconds=10.0)
                    # Do not grant a new recovery window after an earlier timeout.
                    recovery_navigator.started = recovery_started_at
                    state = "WIFI_RECOVERY"
                    print("Fresh Wi-Fi displacement target acquired; leaving WAIT_RAB.", flush=True)
                    if frame_is_new and latest_frame is not None:
                        controller.odom.update(*latest_frame.motor_steps)
                        recovery_navigator.step(latest_frame, math.degrees(controller.odom.theta),
                                                (controller.odom.x, controller.odom.y), wifi_target=target)
                    status = None
                else:
                    status = controller.hold(frame=latest_frame if frame_is_new else None)

        else:
            controller.stop()

        wifi.set_obstacle_avoiding(
            (state == "FOLLOW" and controller_is_locally_avoiding(controller))
            or (state in ("BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB") and recovery_obstacle_active(recovery_navigator))
            or child_avoiding
        )

        if now - last_status >= STATUS_INTERVAL:
            if state in ("FOLLOW", "BLIND_RECOVERY", "WIFI_RECOVERY", "WAIT_RAB"):
                child_age = None if child_id is None else now - last_child
                if status is None:
                    print_mission_status(
                        "self={} role={} front={} child={} phase={} connection_to_front={} age={:.1f}s "
                        "front_mode={} front_slow={} front_hold={} child_ready={} child_status={} child_age={} local=HOLD goal=RUNNING rab_errors={} crc_errors={}".format(
                            robot_name(robot_id),
                            role_name,
                            robot_name(front_id),
                            robot_name(child_id) if child_id is not None else "none",
                            state,
                            current_comm_state,
                            packet_age,
                            front_motion_mode.name,
                            str(front_formation_slow).lower(),
                            str(front_formation_hold).lower(),
                            str(child_ready).lower(),
                            follower_status_name(child_status),
                            "-" if child_age is None else "{:.1f}s".format(child_age),
                            link.i2c_errors,
                            link.crc_errors,
                        )
                    )
                else:
                    print_mission_status(
                        "self={} role={} front={} child={} phase={} connection_to_front={} age={:.1f}s "
                        "front_mode={} front_slow={} front_hold={} child_ready={} child_status={} child_age={} local={} goal=RUNNING bearing={} range={} target_range={} "
                        "range_state={} wheels=({}, {}) rab_errors={} crc_errors={}".format(
                            robot_name(robot_id),
                            role_name,
                            robot_name(front_id),
                            robot_name(child_id) if child_id is not None else "none",
                            state,
                            current_comm_state,
                            packet_age,
                            front_motion_mode.name,
                            str(front_formation_slow).lower(),
                            str(front_formation_hold).lower(),
                            str(child_ready).lower(),
                            follower_status_name(child_status),
                            "-" if child_age is None else "{:.1f}s".format(child_age),
                            status.get("local_state"),
                            _fmt(status.get("bearing")),
                            _fmt(status.get("range")),
                            _fmt(status.get("target_range")),
                            status.get("range_state", "-"),
                            int(status.get("left", 0)),
                            int(status.get("right", 0)),
                            link.i2c_errors,
                            link.crc_errors,
                        )
                    )
            elif state == "CAPTURE":
                print_mission_status("self={} role={} phase=CAPTURE goal=WAITING_TO_SEE_FRONT".format(robot_name(robot_id), role_name))
            elif state == "READY":
                print_mission_status("self={} role={} phase=READY goal=WAITING_FOR_FRONT_START".format(robot_name(robot_id), role_name))
                
            last_status = now

        time.sleep(POLL_INTERVAL)

    controller.stop()
    puck.epuck2.update()

def _fmt(value):
    """Format a numeric telemetry field to two decimals, or '-' if missing."""
    if value is None:
        return "-"
    return "{:+.2f}".format(float(value))


def robot_name(robot_id):
    """Format an integer robot ID as a PiPuck hostname label."""
    return "pipuck{}".format(int(robot_id))


def robot_list(robot_ids):
    """Format a sorted collection of robot IDs for status logging."""
    return "[{}]".format(",".join(robot_name(value) for value in sorted(robot_ids)))


def follower_status_name(status):
    """Map a numeric follower status to a compact log label."""
    status = int(status)
    if status == FOLLOWER_STATUS_AVOIDING:
        return "AVOIDING"
    if status == FOLLOWER_STATUS_HOLDING:
        return "HOLDING"
    return "OK"


def connection_state_from_age(age_seconds):
    """Classify an R&B packet's age as OK, STALE, HOLD or LOST."""
    if age_seconds <= GOOD_PACKET_GAP:
        return "OK"
    if age_seconds <= STALE_PACKET_GAP:
        return "STALE"
    if age_seconds <= LOST_PACKET_GAP:
        return "HOLD"
    return "LOST"


def format_leader_connections(now, follower_ids, last_seen, follower_status):
    """Summarize each follower's link age and last stored status for logs."""
    parts = []
    for follower_id in sorted(follower_ids):
        age = max(0.0, now - last_seen[follower_id])
        parts.append(
            "{}:{} age={:.1f}s status={}".format(
                robot_name(follower_id),
                connection_state_from_age(age),
                age,
                follower_status_name(follower_status.get(follower_id, FOLLOWER_STATUS_NORMAL)),
            )
        )
    if not parts:
        return "none"
    return "; ".join(parts)


def print_mission_status(line):
    """Print a mission telemetry line with the common STATUS prefix."""
    print("STATUS " + line)


def main():
    """Initialize hardware and dispatch the elected mission role.
    
    Parse arguments and robot ID, install shutdown handlers, calibrate R&B,
    discover peers, choose hub or line roles, assign transmit slots, and write
    the launcher readiness marker. Await the start marker, run the role's
    mission loop, then remove markers and stop hardware in a finally block.
    """
    global _stop_requested

    args = parse_args()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    robot_id = robot_id_from_hostname()

    print("R&B-only team mission: stable follow + optional smart election + simple line mode")
    print("Mission version: v31 line READY handshake fix")
    print(
        "robot_id={} team_size={} formation_mode={}".format(
            robot_id,
            args.team_size,
            args.formation_mode,
        )
    )
    print("No Bluetooth server/client is used.")
    print(
        "Leader goal: x={:.1f} mm, y={:.1f} mm".format(
            args.target_x,
            args.target_y,
        )
    )

    with PiPuck() as puck, RangeAndBearing() as rab:
        try:
            puck.epuck2.set_wheel_speeds(0, 0)
            puck.epuck2.update()
            set_all_rgb(puck, 100, 60, 0)

            rab.set_host_calculation(False)
            rab.set_transmission_power(TRANSMISSION_POWER)

            if not wait_for_calibration_trigger(robot_id):
                return

            print("Calibrating ambient light with all transmitters silent.")
            rab.calibrate_light()
            time.sleep(0.50)

            remove_marker(CALIBRATION_TRIGGER)
            remove_marker(CALIBRATION_READY_MARKER)

            link = RabLink(rab, robot_id)
            set_all_rgb(puck, 0, 100, 0)

            team_ids = discover_team(
                robot_id,
                args.team_size,
                link,
            )

            line_order = None
            line_role = None
            line_front_id = None
            line_child_id = None

            if args.formation_mode == "line":
                if args.smart_election:
                    print("Formation mode: LINE. Running local line role detection.")
                    line_decision = run_line_role_election(
                        robot_id,
                        team_ids,
                        link,
                    )
                    line_role = line_decision["role"]
                    line_front_id = line_decision["front_id"]
                    line_child_id = line_decision["child_id"]
                    leader_id = robot_id if line_front_id is None else int(line_front_id)
                    election_mode_name = "LINE_LOCAL_DIRECT"
                    drain_link(link, SMART_DRAIN_SECONDS)
                else:
                    leader_id = min(team_ids)
                    election_mode_name = "SIMPLE_MIN_ID"
                    
                    sorted_ids = sorted(team_ids)
                    idx = sorted_ids.index(robot_id)
                    line_front_id = sorted_ids[idx - 1] if idx > 0 else None
                    line_child_id = sorted_ids[idx + 1] if idx < len(sorted_ids) - 1 else None
                    
                    if line_front_id is None:
                        line_role = "LEADER"
                    elif line_child_id is None:
                        line_role = "FOLLOWER"
                    else:
                        line_role = "LOCAL_LEADER"
                        
                    print("Formation mode: LINE with SIMPLE_MIN_ID. Auto-assigned role: {}".format(line_role))
                    drain_link(link, SMART_DRAIN_SECONDS)
                    
            elif args.smart_election:
                print("Election mode: SMART_FRONT_MOST")
                leader_id = run_smart_election(
                    robot_id,
                    team_ids,
                    link,
                )
                election_mode_name = "SMART_FRONT_MOST"
            else:
                leader_id = min(team_ids)
                election_mode_name = "SIMPLE_MIN_ID"
                print(
                    "Election mode: SIMPLE_MIN_ID. Leader is pipuck{} "
                    "because it has the smallest discovered robot ID.".format(
                        leader_id
                    )
                )
                drain_link(link, SMART_DRAIN_SECONDS)

            if args.formation_mode == "line":
                slots = line_mission_slots(robot_id, team_ids)
            else:
                slots = mission_slots(
                    robot_id,
                    leader_id,
                    team_ids,
                )

            if args.formation_mode == "line":
                if line_role == "LEADER":
                    set_all_rgb(puck, 100, 0, 0)
                    led_name = "RED"
                elif line_role == "LOCAL_LEADER":
                    set_all_rgb(puck, 60, 0, 100)
                    led_name = "PURPLE"
                else:
                    set_all_rgb(puck, 0, 0, 100)
                    led_name = "BLUE"
                role_name = line_role

                print(
                    "Line local decision: role={} front={} child={}".format(
                        role_name,
                        robot_name(line_front_id) if line_front_id is not None else "none",
                        robot_name(line_child_id) if line_child_id is not None else "none",
                    )
                )
                print(
                    "MISSION ROLE: {} is {}; front={}; child={}; LED={}".format(
                        robot_name(robot_id),
                        role_name,
                        robot_name(line_front_id) if line_front_id is not None else "none",
                        robot_name(line_child_id) if line_child_id is not None else "none",
                        led_name,
                    )
                )
            elif robot_id == leader_id:
                set_all_rgb(puck, 100, 0, 0)
                role_name = "LEADER"
                led_name = "RED"
            else:
                set_all_rgb(puck, 0, 0, 100)
                role_name = "FOLLOWER"
                led_name = "BLUE"

            print(
                "Elected leader ID: {} mode={}. ROLE={} LED={}".format(
                    leader_id,
                    election_mode_name,
                    role_name,
                    led_name,
                )
            )
            if args.formation_mode != "line":
                print(
                    "MISSION ROLE: {} is {}; leader={}; followers={}; LED={}".format(
                        robot_name(robot_id),
                        role_name,
                        robot_name(leader_id),
                        robot_list([value for value in team_ids if value != leader_id]),
                        led_name,
                    )
                )
            print("Mission slots: {}".format(sorted(slots)))

            with open(ELECTION_READY_MARKER, "w") as marker:
                marker.write(
                    "robot_id={} team_ids={} leader_id={} formation_mode={} line_order={} role={} front={} child={}\n".format(
                        robot_id,
                        team_ids,
                        leader_id,
                        args.formation_mode,
                        line_order,
                        role_name,
                        line_front_id,
                        line_child_id,
                    )
                )

            wait_for_start_trigger(
                robot_id,
                link,
                team_ids,
            )

            if _stop_requested:
                return

            if args.formation_mode == "line":
                if line_role == "LEADER":
                    set_all_rgb(puck, 100, 0, 0)
                    run_line_leader(
                        puck,
                        robot_id,
                        line_child_id,
                        link,
                        slots,
                        args.target_x,
                        args.target_y,
                    )
                else:
                    if line_role == "LOCAL_LEADER":
                        set_all_rgb(puck, 60, 0, 100)
                    else:
                        set_all_rgb(puck, 0, 0, 100)
                    run_line_follower(
                        puck,
                        robot_id,
                        line_front_id,
                        line_child_id,
                        link,
                        slots,
                    )
            elif robot_id == leader_id:
                set_all_rgb(puck, 100, 0, 0)

                run_leader(
                    puck,
                    robot_id,
                    [
                        value
                        for value in team_ids
                        if value != leader_id
                    ],
                    link,
                    slots,
                    args.target_x,
                    args.target_y,
                )
            else:
                set_all_rgb(puck, 0, 0, 100)

                run_follower(
                    puck,
                    robot_id,
                    leader_id,
                    link,
                    slots,
                )

        finally:
            remove_marker(CALIBRATION_TRIGGER)
            remove_marker(CALIBRATION_READY_MARKER)
            remove_marker(ELECTION_READY_MARKER)
            remove_marker(START_TRIGGER)
            safe_stop(puck)
            print("R&B-only team mission stopped safely.")


if __name__ == "__main__":
    main()