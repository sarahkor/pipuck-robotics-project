#!/bin/bash
set -u

# =========================
# CHOOSE WHICH ROBOTS TO RUN
# =========================

PIPUCK0_ON=true
PIPUCK1_ON=true
PIPUCK2_ON=true

# =========================
# ELECTION MODE
# =========================

# false = simple election: smallest discovered robot ID becomes leader.
# true  = smart election: front-most robot becomes leader.
USE_SMART_ELECTION=true

# Choose: hub or line
# hub  = old behavior: all followers follow the elected leader
# line = already-formed chain: front -> middle/local leader -> back follower
FORMATION_MODE="line"

# =========================
# ROBOT HOSTS
# =========================

PIPUCK0_HOST="pipuck0.local"
PIPUCK1_HOST="pipuck1.local"
PIPUCK2_HOST="pipuck2.local"

# =========================
# PROJECT DIRECTORIES ON ROBOTS
# =========================

ROBOT_DIR="/home/pi/pipuck-walk"
LOG_FILE="team_mission_rab.log"

TARGET_X="-250"
TARGET_Y="550"

CALIBRATION_READY_MARKER="RAB_CALIBRATION_READY"
CALIBRATION_TRIGGER="RAB_CALIBRATE"
ELECTION_READY_MARKER="RAB_ELECTION_READY"
START_TRIGGER="RAB_TEAM_START"

ACTIVE_HOSTS=()
ACTIVE_NAMES=()
ACTIVE_SOURCE_DIRS=()

if [ "$PIPUCK0_ON" = true ]; then
    ACTIVE_HOSTS+=("$PIPUCK0_HOST")
    ACTIVE_NAMES+=("pipuck0")
    ACTIVE_SOURCE_DIRS+=("rob0")
fi

if [ "$PIPUCK1_ON" = true ]; then
    ACTIVE_HOSTS+=("$PIPUCK1_HOST")
    ACTIVE_NAMES+=("pipuck1")
    ACTIVE_SOURCE_DIRS+=("rob1")
fi

if [ "$PIPUCK2_ON" = true ]; then
    ACTIVE_HOSTS+=("$PIPUCK2_HOST")
    ACTIVE_NAMES+=("pipuck2")
    ACTIVE_SOURCE_DIRS+=("rob2")
fi

TEAM_SIZE=${#ACTIVE_HOSTS[@]}

if [ "$TEAM_SIZE" -lt 2 ]; then
    echo "At least two robots must be enabled."
    exit 1
fi

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

stop_robot() {
    local host=$1
    echo "Stopping old R&B mission on $host..."
    ssh_robot "$host" "pkill -TERM -f '[t]eam_mission_rab.py' || true; sleep 0.3; pkill -KILL -f '[t]eam_mission_rab.py' || true; cd '$ROBOT_DIR' 2>/dev/null && rm -f '$CALIBRATION_READY_MARKER' '$CALIBRATION_TRIGGER' '$ELECTION_READY_MARKER' '$START_TRIGGER' || true"
}

force_safe_robot() {
    local host=$1
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
}

source_file() {
    local src_dir=$1
    local name=$2

    if [ -f "$src_dir/$name" ]; then
        echo "$src_dir/$name"
    elif [ -f "$name" ]; then
        echo "$name"
    else
        echo ""
    fi
}

upload_robot() {
    local host=$1
    local src_dir=$2

    echo "Uploading stable R&B mission files to $host from $src_dir..."
    ssh_robot "$host" "mkdir -p '$ROBOT_DIR'"

    local files=()
    local f

    for name in \
        team_mission_rab.py \
        follow_rob.py \
        reach_goal_rab.py \
        range_and_bearing_protocol.py \
        range_and_bearing.py \
        range_and_bearing_filter.py \
        odometry.py
    do
        f=$(source_file "$src_dir" "$name")
        if [ -n "$f" ]; then
            files+=("$f")
        fi
    done

    if [ "${#files[@]}" -eq 0 ]; then
        echo "No files found to upload for $host."
        exit 1
    fi

    scp -q "${SSH_OPTS[@]}" "${files[@]}" "pi@$host:$ROBOT_DIR/"
}

start_robot() {
    local host=$1
    local smart_arg=""

    if [ "$USE_SMART_ELECTION" = true ]; then
        smart_arg=" --smart-election"
    fi

    echo "Starting stable R&B mission on $host..."
    ssh_robot "$host" "cd '$ROBOT_DIR' && rm -f '$CALIBRATION_READY_MARKER' '$CALIBRATION_TRIGGER' '$ELECTION_READY_MARKER' '$START_TRIGGER' '$LOG_FILE' && setsid -f python3 -u team_mission_rab.py --team-size '$TEAM_SIZE' --target-x '$TARGET_X' --target-y '$TARGET_Y' --formation-mode '$FORMATION_MODE'$smart_arg > '$LOG_FILE' 2>&1 < /dev/null"
}

wait_for_marker() {
    local host=$1
    local marker=$2
    local label=$3
    local deadline=$((SECONDS + 45))

    while [ "$SECONDS" -lt "$deadline" ]; do
        if ssh_robot "$host" "test -f '$ROBOT_DIR/$marker'"; then
            echo "$host: $label"
            return 0
        fi
        sleep 0.5
    done

    echo "$host: timed out waiting for $marker"
    ssh_robot "$host" "tail -n 100 '$ROBOT_DIR/$LOG_FILE' 2>/dev/null || true"
    return 1
}

read_robot_dashboard() {
    local host=$1
    ssh_robot "$host" "cd '$ROBOT_DIR' 2>/dev/null || exit 0
if pgrep -f '[t]eam_mission_rab.py' >/dev/null 2>&1; then
    echo 'process=RUNNING'
else
    echo 'process=STOPPED'
fi
if [ -f '$LOG_FILE' ]; then
    grep -E 'MISSION ROLE:|STATUS |Election mode:|SMART FINAL:|SMART RESULT:|SMART CANDIDATE:|SMART FACING:|SMART POSITIONS:|All followers READY|Goal reached|Leader mission stopped|Leader stopped|Follower .*TEAM STOP|Lost R&B contact|Leader missing|R&B-only team mission stopped safely' '$LOG_FILE' | tail -n 10 || true
else
    echo 'log=missing'
fi"
}

show_status_loop() {
    echo ""
    echo "Mission dashboard. Press Ctrl+C to stop robots safely."
    echo "Dashboard shows: role, leader/follower, connection health, goal state, and stop/reached events."
    echo ""

    local status_dir
    status_dir=$(mktemp -d 2>/dev/null || mktemp -d -t rab_status)

    while true; do
        local any_running=false
        local i=0
        local snapshot_file="$status_dir/current.txt"
        : > "$snapshot_file"

        for host in "${ACTIVE_HOSTS[@]}"; do
            local name=${ACTIVE_NAMES[$i]}
            echo "--- $name ---" >> "$snapshot_file"

            local robot_status
            robot_status=$(read_robot_dashboard "$host")
            echo "$robot_status" >> "$snapshot_file"

            if echo "$robot_status" | grep -q '^process=RUNNING$'; then
                any_running=true
            fi
            i=$((i + 1))
        done

        local previous_file="$status_dir/previous.txt"
        if [ ! -f "$previous_file" ] || ! cmp -s "$snapshot_file" "$previous_file"; then
            echo "[$(date +%H:%M:%S)]"
            cat "$snapshot_file"
            echo ""
            cp "$snapshot_file" "$previous_file"
        fi

        if [ "$any_running" = false ]; then
            echo "[$(date +%H:%M:%S)] All robot mission processes have exited. Dashboard stopped."
            rm -rf "$status_dir"
            return 0
        fi

        sleep 2
    done
}

cleanup() {
    echo ""
    echo "Stopping stable R&B mission and turning robots off..."
    for host in "${ACTIVE_HOSTS[@]}"; do
        stop_robot "$host"
    done
    for host in "${ACTIVE_HOSTS[@]}"; do
        force_safe_robot "$host"
    done
    echo "All active robots stopped, motors off, LEDs off."
}

cleanup_and_exit() {
    cleanup
    exit 130
}

trap cleanup_and_exit INT TERM

# =========================
# MAIN
# =========================

echo "Stable R&B mission launcher with optional smart election"
echo "Active team size: $TEAM_SIZE"
echo "Target: x=$TARGET_X y=$TARGET_Y"
echo "Use smart election: $USE_SMART_ELECTION"
echo "Formation mode: $FORMATION_MODE"
echo ""

for host in "${ACTIVE_HOSTS[@]}"; do
    stop_robot "$host"
done

for host in "${ACTIVE_HOSTS[@]}"; do
    force_safe_robot "$host"
done

for i in "${!ACTIVE_HOSTS[@]}"; do
    upload_robot "${ACTIVE_HOSTS[$i]}" "${ACTIVE_SOURCE_DIRS[$i]}"
done

for host in "${ACTIVE_HOSTS[@]}"; do
    start_robot "$host"
done

echo "Waiting for calibration-ready markers..."
for host in "${ACTIVE_HOSTS[@]}"; do
    wait_for_marker "$host" "$CALIBRATION_READY_MARKER" "ready for calibration" || { cleanup; exit 1; }
done

echo "Triggering synchronized R&B ambient calibration..."
for host in "${ACTIVE_HOSTS[@]}"; do
    ssh_robot "$host" "cd '$ROBOT_DIR' && touch '$CALIBRATION_TRIGGER'"
done

echo "Waiting for election-ready markers..."
for host in "${ACTIVE_HOSTS[@]}"; do
    wait_for_marker "$host" "$ELECTION_READY_MARKER" "leader elected / mission ready" || { cleanup; exit 1; }
done

echo "Starting mission on all robots..."
for host in "${ACTIVE_HOSTS[@]}"; do
    ssh_robot "$host" "cd '$ROBOT_DIR' && touch '$START_TRIGGER'"
done

show_status_loop
cleanup
