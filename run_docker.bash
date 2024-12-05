#!/bin/bash
# =============================================================================
# Unitree Go1 ROS Bridge Docker Image Run
# Created by: Shaun Altmann
#
# Used to run the pre-made docker image with the required settings.
# =============================================================================


# =============================================================================
# Define Container Name
# =============================================================================
CONTAINER_NAME="ros_bridge"


# =============================================================================
# Delete the Pre-Existing Docker Container
# =============================================================================
delete_old_container() {
    docker rm -f $CONTAINER_NAME 2> /dev/null # hide error message
}


# =============================================================================
# Create a New Docker Image Tab
# =============================================================================
new_tab() {
    docker exec -it $CONTAINER_NAME bash
}


# =============================================================================
# Run the Docker Image
# =============================================================================
run_docker() {
    local add_host=false

    # parse run-time arguments
    while [[ "$#" -gt 0 ]]; do
        case "$1" in
            --add-host) add_host=true;;
            *) echo "Invalid Option: $1"; return 1;;
        esac
        shift
    done

    cmd="docker run -it --net=host --ipc=host --name=$CONTAINER_NAME"

    if $add_host; then
        cmd="$cmd --add-host=raspberrypi:192.168.123.161"
    fi

    eval "$cmd ros-bridge"
}


# =============================================================================
# Print Script Usage
# =============================================================================
usage() {
    echo "Usage: $0 [-h] [-n name] [-t]"
    echo ""
    echo "Purpose:"
    echo "    This script runs a Docker image with the required settings for "
    echo "    the Unitree Go1 ROS Bridge."
    echo ""
    echo "Options:"
    echo "    -h        Print this help message."
    echo "    -n name   Specify the container name (default: ros_bridge)."
    echo "    -p        Use if running with the physical Unitree Go1 Robot."
    echo "    -t        Run the container from a new terminal tab. This "
    echo "              requires that the container has already been created."
}


# =============================================================================
# Handle Arguments
# =============================================================================
ADD_HOST=false
while getopts "hn:pt" opt; do
    case "$opt" in
        h)
            usage
            exit 0
            ;;
        n)
            CONTAINER_NAME="$OPTARG"
            ;;
        t)
            new_tab
            exit 0
            ;;
        p)
            ADD_HOST=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
    esac
done
delete_old_container
if $ADD_HOST; then
    run_docker --add-host
else
    run_docker
fi
exit 0


# =============================================================================
# End of File
# =============================================================================
