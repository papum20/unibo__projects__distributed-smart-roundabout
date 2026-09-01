#!/bin/bash

set -euo pipefail

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 {PAUSE|RESUME|p|r|pause|resume}" >&2
    exit 1
fi

raw="$1"
cmd="${raw^^}"   # convert to uppercase

case "$cmd" in
    P|PAUSE)
        cmd="PAUSE"
        ;;
    R|RESUME)
        cmd="RESUME"
        ;;
    *)
        echo "Error: only PAUSE or RESUME are allowed (also p/r)." >&2
        exit 1
        ;;
esac

curl -sS -X POST http://localhost:8080/api/control \
    -H "Content-Type: application/json" \
    -d "{\"command\": \"${cmd}\"}"