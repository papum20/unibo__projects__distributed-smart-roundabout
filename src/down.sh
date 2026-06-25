#!/bin/bash

if [ -z src/ ]; then
	echo "Error: This script must be run from the project root directory"
	exit 1
fi

set -a
source src/bash-scripts/scripts.env

docker compose -f $DOCKER_COMPOSE_FILE down