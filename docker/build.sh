#!/usr/bin/bash

set -e

# Change to parent directory
cd ..

# Check for --no-cache flag
NO_CACHE_FLAG=""
if [ "$1" = "--no-cache" ]; then
    NO_CACHE_FLAG="--no-cache"
fi

# Build the image from parent directory
docker build $NO_CACHE_FLAG -t bag_converter:latest -f docker/Dockerfile . --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g)