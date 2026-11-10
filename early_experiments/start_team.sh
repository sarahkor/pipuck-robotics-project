#!/bin/bash

# =========================
# CHOOSE WHICH ROBOTS TO RUN
# =========================

PIPUCK0_ON=true
PIPUCK1_ON=false
PIPUCK2_ON=true

# =========================
# ROBOT HOSTS
# =========================

PIPUCK0_HOST="pipuck0.local"
PIPUCK1_HOST="pipuck1.local"
PIPUCK2_HOST="pipuck2.local"

# =========================
# PROJECT DIRECTORIES ON ROBOTS
# =========================

PIPUCK0_DIR="/home/pi/pipuck-walk"
PIPUCK1_DIR="/home/pi/pipuck-walk"
PIPUCK2_DIR="/home/pi/pipuck-walk"

# =========================
# HELPERS
# =========================

stop_robot() {
    local host=$1
    echo "Stopping old team_mission on $host..."
    ssh pi@$host "pkill -f 'team_mission.py' || true"
}

start_robot() {
    local host=$1
    local dir=$2
    echo "Starting team_mission on $host..."
    ssh pi@$host "cd $dir && nohup python3 -u team_mission.py > team_mission.log 2>&1 < /dev/null &"
}

# =========================
# STOP OLD RUNS
# =========================

echo "Stopping old team_mission processes..."

if [ "$PIPUCK0_ON" = true ]; then
    stop_robot "$PIPUCK0_HOST"
fi

if [ "$PIPUCK1_ON" = true ]; then
    stop_robot "$PIPUCK1_HOST"
fi

if [ "$PIPUCK2_ON" = true ]; then
    stop_robot "$PIPUCK2_HOST"
fi

sleep 1

# =========================
# START ROBOTS
# =========================

# Start server first
if [ "$PIPUCK0_ON" = true ]; then
    start_robot "$PIPUCK0_HOST" "$PIPUCK0_DIR"
    sleep 4
fi

# Start clients after server
if [ "$PIPUCK1_ON" = true ]; then
    start_robot "$PIPUCK1_HOST" "$PIPUCK1_DIR"
    sleep 1
fi

if [ "$PIPUCK2_ON" = true ]; then
    start_robot "$PIPUCK2_HOST" "$PIPUCK2_DIR"
    sleep 1
fi

echo ""
echo "Selected robots started."
echo ""

if [ "$PIPUCK0_ON" = true ]; then
    echo "Watch pipuck0:"
    echo "ssh pi@$PIPUCK0_HOST 'tail -f $PIPUCK0_DIR/team_mission.log'"
    echo ""
fi

if [ "$PIPUCK1_ON" = true ]; then
    echo "Watch pipuck1:"
    echo "ssh pi@$PIPUCK1_HOST 'tail -f $PIPUCK1_DIR/team_mission.log'"
    echo ""
fi

if [ "$PIPUCK2_ON" = true ]; then
    echo "Watch pipuck2:"
    echo "ssh pi@$PIPUCK2_HOST 'tail -f $PIPUCK2_DIR/team_mission.log'"
    echo ""
fi