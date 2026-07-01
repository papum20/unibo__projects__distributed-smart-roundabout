#!/bin/bash

# Usage: ./cleanup-env.sh [input_file] [output_file]
# Default: ./cleanup-env.sh .env .env.clean

set -a
source src/bash-scripts/scripts.env

input_file="${1:-$DOCKER_ENV}"
output_file="${2:-$CLEAN_ENV}"

if [ ! -f "$input_file" ]; then
    echo "Error: File '$input_file' not found"
    exit 1
fi

# Remove all spaces/tabs before and after = sign
sed 's/[[:space:]]*=[[:space:]]*/=/g' "$input_file" > "$output_file"

echo "Successfully cleaned '$input_file'"
echo "Output written to: $output_file"