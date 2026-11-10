"""Exchange robot pose estimates and obstacle status over Wi-Fi UDP broadcasts.

Each robot broadcasts its own odometry coordinates and stores the latest
received state per teammate. Receivers use local monotonic timestamps to
decide whether a teammate update is still fresh.
"""
import json
import math
import socket
import threading
import time

WIFI_PORT = 5005
BROADCAST_IP = '<broadcast>'


class WorldStateNetwork:
    """Broadcast local state and cache fresh state received from teammates.

    A daemon thread sends JSON UDP broadcasts; a second thread receives them.
    Pose values come from the supplied odometry object. The class does not
    calculate or align odometry coordinate frames.
    """
    def __init__(self, robot_id, role, odometry):
        """Configure the UDP socket and shared teammate-state cache.

        Args:
            robot_id: This robot's numeric identifier, used to ignore its own broadcasts.
            role: Role label included in outgoing status messages.
            odometry: Object exposing ``x``, ``y``, and ``theta`` coordinates.

        The socket is bound to ``WIFI_PORT`` with a receive timeout of 0.5 s.
        Network threads are started separately by :meth:`start`.
        """
        self.robot_id = int(robot_id)
        self.role = role
        self.odometry = odometry
        self.teammates_state = {}
        self.running = True
        self.obstacle_avoiding = False
        self._lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.bind(('', WIFI_PORT))
        self.sock.settimeout(0.5)

    def start(self):
        """Start daemon threads for periodic broadcasts and incoming packets.

        This method does not block or wait for a teammate to connect.
        """
        threading.Thread(target=self._broadcast_loop, daemon=True).start()
        threading.Thread(target=self._listen_loop, daemon=True).start()

    def set_obstacle_avoiding(self, active):
        """Set the obstacle-avoidance flag published in future broadcasts.

        Args:
            active: Truthy when this robot is actively avoiding an obstacle.
        """
        with self._lock:
            self.obstacle_avoiding = bool(active)

    def _broadcast_loop(self):
        """Send this robot's current pose and avoidance flag every ~0.2 s.

        Position and heading are read directly from ``odometry``; heading is
        in the odometry object's native units (radians in this project).
        Transmission errors are ignored so the loop can try again later.
        """
        while self.running:
            with self._lock:
                avoiding = self.obstacle_avoiding
            payload = {
                'id': self.robot_id, 'role': self.role,
                'x': round(self.odometry.x, 2),
                'y': round(self.odometry.y, 2),
                'theta': round(self.odometry.theta, 2),
                'obstacle_avoiding': avoiding,
            }
            try:
                self.sock.sendto(json.dumps(payload).encode('utf-8'),
                                 (BROADCAST_IP, WIFI_PORT))
            except OSError:
                pass
            time.sleep(0.2)

    def _listen_loop(self):
        """Receive JSON broadcasts and update the latest state for each peer.

        Ignore messages from this robot, malformed messages, and non-finite
        position/heading values. Accepted packets are timestamped with the
        receiver's monotonic clock for subsequent freshness checks.
        """
        while self.running:
            try:
                raw, _ = self.sock.recvfrom(1024)
                state = json.loads(raw.decode('utf-8'))
                rid = int(state['id'])
                if rid == self.robot_id:
                    continue
                state = {'id': rid, 'role': state['role'],
                         'x': float(state['x']), 'y': float(state['y']),
                         'theta': float(state['theta']),
                         'obstacle_avoiding': state.get('obstacle_avoiding') is True,
                         '_received_mono': time.monotonic()}
                if not all(math.isfinite(state[k]) for k in ('x', 'y', 'theta')):
                    continue
                with self._lock:
                    self.teammates_state[rid] = state
            except (ValueError, KeyError, TypeError, UnicodeDecodeError, socket.timeout):
                pass
            except OSError:
                if not self.running:
                    break

    def get_fresh_state(self, robot_id, max_age=1.2):
        """Return a copy of a teammate's latest state if sufficiently recent.

        Args:
            robot_id: Identifier of the teammate to look up.
            max_age: Maximum permitted age in seconds since local reception.

        Returns:
            A shallow copy of the cached state dictionary, or ``None`` when
            no state exists or the last packet is too old.
        """
        with self._lock:
            state = self.teammates_state.get(int(robot_id))
            if state is None or time.monotonic() - state['_received_mono'] > max_age:
                return None
            return state.copy()

    def stop(self):
        """Request both daemon loops to stop and close the UDP socket."""
        self.running = False
        self.sock.close()
