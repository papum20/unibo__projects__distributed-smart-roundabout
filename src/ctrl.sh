#!/bin/bash

set -euo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Usage: $0 {PAUSE|RESUME|FAILSAFE|CONTROLLER|DISCONNECT|RECONNECT} [vehicle-id]" >&2
    exit 1
fi

raw="${1^^}"    # convert to uppercase

case "$raw" in
    P|PAUSE)
        cmd="PAUSE"
        ;;
    R|RESUME)
        cmd="RESUME"
        ;;
    F|FAILSAFE)
        cmd="ENTER_FAILSAFE"
        ;;
    C|CONTROLLER)
        cmd="EXIT_FAILSAFE"
        ;;
    D|DISCONNECT)
        cmd="ENTER_DISCONNECTED"
        ;;
    N|RECONNECT)
        cmd="EXIT_DISCONNECTED"
        ;;
    *)
        echo "Error: unsupported command." >&2
        exit 1
        ;;
esac

if [ "$#" -eq 2 ]; then
    vehicle_id="$2"
    payload=$(printf '{"command":"%s","vehicle_id":"%s"}' "$cmd" "$vehicle_id")
else
    payload=$(printf '{"command":"%s"}' "$cmd")
fi

curl -sS -X POST http://localhost:8080/api/control \
    -H "Content-Type: application/json" \
    -d "$payload"