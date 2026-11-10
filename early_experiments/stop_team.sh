#!/bin/bash

PIPUCK0_ON=true
PIPUCK1_ON=true
PIPUCK2_ON=true

PIPUCK0_HOST="pipuck0.local"
PIPUCK1_HOST="pipuck1.local"
PIPUCK2_HOST="pipuck2.local"

stop_robot() {
    local host=$1
    echo "Stopping team_mission on $host..."
    ssh pi@$host "pkill -f 'team_mission.py' || true"
}

if [ "$PIPUCK0_ON" = true ]; then
    stop_robot "$PIPUCK0_HOST"
fi

if [ "$PIPUCK1_ON" = true ]; then
    stop_robot "$PIPUCK1_HOST"
fi

if [ "$PIPUCK2_ON" = true ]; then
    stop_robot "$PIPUCK2_HOST"
fi

echo "Stopped selected robots."