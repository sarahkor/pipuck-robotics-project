#!/bin/bash
set -u

PIPUCK0_ON=true
PIPUCK1_ON=true
PIPUCK2_ON=true

PIPUCK0_HOST="pipuck0.local"
PIPUCK1_HOST="pipuck1.local"
PIPUCK2_HOST="pipuck2.local"
ROBOT_DIR="/home/pi/pipuck-walk"

ACTIVE_HOSTS=()

if [ "$PIPUCK0_ON" = true ]; then ACTIVE_HOSTS+=("$PIPUCK0_HOST"); fi
if [ "$PIPUCK1_ON" = true ]; then ACTIVE_HOSTS+=("$PIPUCK1_HOST"); fi
if [ "$PIPUCK2_ON" = true ]; then ACTIVE_HOSTS+=("$PIPUCK2_HOST"); fi

SSH_OPTS=(
    -q
    -o LogLevel=ERROR
    -o BatchMode=yes
    -o ConnectTimeout=10
)

ssh_robot() {
    local host=$1
    shift
    ssh -n "${SSH_OPTS[@]}" "pi@$host" "$@"
}

for host in "${ACTIVE_HOSTS[@]}"; do
    echo "Stopping stable R&B mission on $host..."
    ssh_robot "$host" "pkill -TERM -f '[t]eam_mission_rab.py' || true; sleep 0.4; pkill -KILL -f '[t]eam_mission_rab.py' || true; cd '$ROBOT_DIR' 2>/dev/null && rm -f RAB_CALIBRATION_READY RAB_CALIBRATE RAB_ELECTION_READY RAB_TEAM_START || true"
    ssh_robot "$host" "python3 - <<'PY'
try:
    from pipuck import PiPuck
    with PiPuck() as puck:
        puck.epuck2.set_wheel_speeds(0, 0)
        for idx in (2, 4, 6, 8):
            puck.epuck2.set_rgb_led(idx, 0, 0, 0)
        puck.epuck2.update()
except Exception as error:
    print('safe stop warning:', error)
PY"
    echo "$host stopped."
done

echo "Stopped selected robots, motors off, LEDs off."
