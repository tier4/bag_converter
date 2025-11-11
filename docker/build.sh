#!/usr/bin/bash

set -e

# Change to parent directory
cd ..

# Build the Docker image
echo "Building bag_converter Docker image..."

# Build the image from parent directory
docker build -t bag_converter:latest -f docker/Dockerfile . --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) --no-cache

echo "Docker image built successfully: bag_converter:latest"
