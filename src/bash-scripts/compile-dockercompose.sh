#!/bin/bash

if [ -z src/ ]; then
	echo "Error: src/ not found; run this script from the root directory."
	exit 1
fi

set -a 
source src/bash-scripts/scripts.env
./src/bash-scripts/env-cleanup.sh

set -a
source src/bash-scripts/clean.env.generated

#export DOLLAR='$'
#envsubst < $DOCKER_COMPOSE_TEMPLATE > $DOCKER_COMPOSE_FILE
#echo "Successfully compiled '$DOCKER_COMPOSE_TEMPLATE' to '$DOCKER_COMPOSE_FILE'"
