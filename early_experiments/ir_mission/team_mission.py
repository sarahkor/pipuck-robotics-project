"""Coordinate a two-robot PiPuck mission over Bluetooth RFCOMM.

The robots establish a Bluetooth connection, elect a leader by comparing
random integers, and execute different missions based on the elected role.
The follower sends a one-byte READY signal before the leader begins moving.

This is a standalone Bluetooth experiment, separate from the R&B/Wi-Fi
mission implemented in ``team_mission_rab.py``.
"""

import socket
import random
from time import sleep

from pipuck import PiPuck

from reach_goal import StateMachine as LeaderMission
from follow_obj import HandFollower as FollowerMission


# Connection settings are configured independently on each robot.
ROBOT_NAME = "robot_B"

# Exactly one robot must accept the connection; this does not elect the leader.
IS_SERVER = False

PORT = 1

# Bluetooth MAC of the robot acting as the RFCOMM server.
SERVER_MAC = "B8:27:EB:EB:D8:DD"

# Local address on which the RFCOMM server listens.
SERVER_BIND_MAC = SERVER_MAC

READY_BYTE = b"R"


def send_line(sock, text):
    """Send a UTF-8 text message terminated by a newline.

    Args:
        sock: Connected Bluetooth stream socket.
        text: Text to send to the peer.
    """
    sock.sendall((text + "\n").encode())


def recv_line(sock):
    """Receive a newline-terminated message from a Bluetooth peer.

    Reads one byte at a time until a newline arrives. Raises RuntimeError
    if the connection closes before a complete line is received.

    Args:
        sock: Connected Bluetooth stream socket.

    Returns:
        Decoded text with leading and trailing whitespace removed.
    """
    data = b""

    while not data.endswith(b"\n"):
        chunk = sock.recv(1)

        if not chunk:
            raise RuntimeError("Bluetooth connection closed")

        data += chunk

    return data.decode().strip()


def wait_for_ready(sock):
    """Block the elected leader until the follower sends the READY byte.

    Raises RuntimeError if the Bluetooth connection closes while waiting.
    Other incoming bytes are logged and ignored.

    Args:
        sock: Connected Bluetooth stream socket.
    """
    print("Leader is waiting for follower READY signal...")

    while True:
        data = sock.recv(1)

        if not data:
            raise RuntimeError("Bluetooth connection closed while waiting for READY")

        if data == READY_BYTE:
            print("READY received from follower. Leader starts moving.")
            return

        print("Leader received unknown byte: {}".format(data))


def make_ready_callback(sock):
    """Create a callback that sends the follower's READY signal once.

    The callback captures a mutable flag so repeated calls do not send
    duplicate READY bytes. It is passed to ``FollowerMission``.

    Args:
        sock: Connected Bluetooth stream socket.

    Returns:
        A zero-argument function that sends READY on its first invocation.
    """
    state = {"sent": False}

    def callback():
        """Signal follower readiness on the first invocation only."""
        if state["sent"]:
            return

        print("Follower is ready. Sending READY byte to leader...")
        sock.sendall(READY_BYTE)
        state["sent"] = True
        print("READY sent.")

    return callback


def run_election_server():
    """Accept a Bluetooth connection and run the server side of election.

    In each round both robots independently choose an integer in [0, 100].
    The lower value wins; ties cause another round. The server receives the
    client's value first, then reports the result over Bluetooth.

    Returns:
        ``(is_leader, client_socket, server_socket)``; both sockets remain
        open for the subsequent mission and must later be closed by the caller.

    Raises:
        Exception: Propagates connection or election errors after closing
            sockets already created by this function.
    """
    server_sock = socket.socket(
        socket.AF_BLUETOOTH,
        socket.SOCK_STREAM,
        socket.BTPROTO_RFCOMM
    )

    client_sock = None

    try:
        server_sock.bind((SERVER_BIND_MAC, PORT))
        server_sock.listen(1)

        print("{} is listening over Bluetooth...".format(ROBOT_NAME))

        client_sock, address = server_sock.accept()
        print("Connected to peer at MAC: {}".format(address[0]))

        round_num = 1

        while True:
            my_number = random.randint(0, 100)

            peer_msg = recv_line(client_sock)
            peer_number = int(peer_msg)

            print("")
            print("Election round {}".format(round_num))
            print("My random number: {}".format(my_number))
            print("Peer random number: {}".format(peer_number))

            if my_number == peer_number:
                print("Tie. Repeating election.")
                send_line(client_sock, "TIE")
                round_num += 1
                continue

            if my_number < peer_number:
                print("I won the election.")
                send_line(client_sock, "SERVER")
                return True, client_sock, server_sock
            else:
                print("I lost the election.")
                send_line(client_sock, "CLIENT")
                return False, client_sock, server_sock

    except Exception:
        if client_sock is not None:
            client_sock.close()
        server_sock.close()
        raise


def run_election_client():
    """Connect to the RFCOMM server and run the client side of election.

    In each round the client sends its random integer and reads the server's
    result. A tie starts another round. The connection remains open on success.

    Returns:
        ``(is_leader, client_socket, None)``.

    Raises:
        Exception: Propagates connection or election errors after closing
            the client's socket.
    """
    client_sock = socket.socket(
        socket.AF_BLUETOOTH,
        socket.SOCK_STREAM,
        socket.BTPROTO_RFCOMM
    )

    try:
        print("{} is connecting to server...".format(ROBOT_NAME))
        client_sock.connect((SERVER_MAC, PORT))
        print("Connected to server.")

        round_num = 1

        while True:
            my_number = random.randint(0, 100)

            print("")
            print("Election round {}".format(round_num))
            print("My random number: {}".format(my_number))

            send_line(client_sock, str(my_number))

            result = recv_line(client_sock)
            print("Election result message: {}".format(result))

            if result == "TIE":
                print("Tie. Repeating election.")
                round_num += 1
                continue

            if result == "CLIENT":
                print("I won the election.")
                return True, client_sock, None

            if result == "SERVER":
                print("I lost the election.")
                return False, client_sock, None

            raise RuntimeError("Unknown election result: {}".format(result))

    except Exception:
        client_sock.close()
        raise


def run_leader_election():
    """Run the election using this robot's configured Bluetooth socket role.

    ``IS_SERVER`` selects the connection role, not the elected mission role.

    Returns:
        The election result and sockets from the server or client routine.
    """
    if IS_SERVER:
        return run_election_server()
    else:
        return run_election_client()


def main():
    """Elect mission roles, run the selected robot mission, and clean up.

    The leader waits for a READY signal and runs ``LeaderMission``; the
    follower runs ``FollowerMission`` with a one-shot READY callback.
    Cleanup stops the mission where supported and closes Bluetooth sockets.
    """
    print("Starting team mission.")
    print("Robot name: {}".format(ROBOT_NAME))
    print("IS_SERVER: {}".format(IS_SERVER))

    i_am_leader, bt_sock, server_sock = run_leader_election()

    if i_am_leader:
        print("")
        print("ROLE: LEADER")
        print("I will wait for follower READY, then run reach_goal.py.")
    else:
        print("")
        print("ROLE: FOLLOWER")
        print("I will face the leader, send READY, then run follow_obj.py.")

    puck = PiPuck()
    brain = None

    try:
        with puck:
            try:
                if i_am_leader:
                    wait_for_ready(bt_sock)

                    brain = LeaderMission(puck)
                    brain.run_mission()

                else:
                    ready_callback = make_ready_callback(bt_sock)

                    brain = FollowerMission(
                        puck,
                        ready_callback=ready_callback,
                        search_before_follow=True
                    )
                    brain.run_mission()

            except KeyboardInterrupt:
                print("")
                print("KeyboardInterrupt received. Stopping safely.")

            finally:
                if brain is not None:
                    if hasattr(brain, "cleanup"):
                        brain.cleanup()
                    elif hasattr(brain, "stop_motors"):
                        brain.stop_motors()
                else:
                    puck.epuck2.set_wheel_speeds(0, 0)
                    puck.epuck2.update()

    finally:
        try:
            bt_sock.close()
        except Exception:
            pass

        if server_sock is not None:
            try:
                server_sock.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
