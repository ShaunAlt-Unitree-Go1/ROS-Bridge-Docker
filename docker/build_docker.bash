#!/bin/bash
# =============================================================================
# Unitree Go1 ROS Bridge Docker Image Builder
# Created by: Shaun Altmann
#
# Used to build the docker image with the required settings.
# =============================================================================


# =============================================================================
# Get Script Directory
# =============================================================================
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"


# =============================================================================
# Build the Docker Image
# =============================================================================
docker build \
    --progress=plain \
    -t ros-bridge \
    $SCRIPT_DIR/.


# =============================================================================
# End of File
# =============================================================================
