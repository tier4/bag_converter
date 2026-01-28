#!/usr/bin/bash

set -e

show_help() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Build the bag_converter Docker image.

Options:
    -h, --help              Show this help message and exit
    -j, --parallel-jobs N   Set number of parallel build jobs (default: 4)
    -t, --tag TAG           Set Docker image tag (default: latest)
    --no-cache              Build without using Docker cache

Examples:
    $(basename "$0")                    # Build with default settings
    $(basename "$0") -j 8               # Build with 8 parallel jobs
    $(basename "$0") -t v1.0.0          # Build with tag v1.0.0
    $(basename "$0") --no-cache -j 8    # Build without cache, 8 parallel jobs
EOF
}

# Change to parent directory
cd "$(dirname "$0")/.."

# Default values
NO_CACHE_FLAG=""
PARALLEL_JOBS=4
TAG=latest

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
    -h | --help)
        show_help
        exit 0
        ;;
    --no-cache)
        NO_CACHE_FLAG="--no-cache"
        shift
        ;;
    --parallel-jobs | -j)
        PARALLEL_JOBS="$2"
        shift 2
        ;;
    --tag | -t)
        TAG="$2"
        shift 2
        ;;
    *)
        echo "Unknown option: $1"
        echo "Run '$(basename "$0") --help' for usage."
        exit 1
        ;;
    esac
done

# Build the image from parent directory
docker build $NO_CACHE_FLAG -t bag_converter:"${TAG}" -f docker/Dockerfile . \
    --build-arg USER_ID="$(id -u)" \
    --build-arg GROUP_ID="$(id -g)" \
    --build-arg PARALLEL_JOBS="${PARALLEL_JOBS}" \
    --build-arg VERSION="${TAG}"
