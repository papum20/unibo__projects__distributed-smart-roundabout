#!/bin/bash

curl -sS http://localhost:8080/api/state |
jq -c --arg prefix "$1" '
  .vehicles
  | to_entries[]
  | select(.key | startswith($prefix))
  | .value
'