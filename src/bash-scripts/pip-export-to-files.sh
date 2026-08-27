#!/bin/bash

if [ -z src/ ]; then
	echo "Error: src/ not found; run this script from the root directory."
	exit 1
fi

pip list --format=columns | tail -n +3 | awk '{print $1"==" $2}' | tee \
	src/service-controller/requirements.txt \
	src/service-vehicle/requirements.txt \
	src/service-webviewer/requirements.txt