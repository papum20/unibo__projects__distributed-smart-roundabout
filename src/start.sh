#!/bin/bash

if [ -z src/ ]; then
	echo "Error: This script must be run from the project root directory"
	exit 1
fi

docker compose -f src/docker-compose.yml --env-file src/.env up --build