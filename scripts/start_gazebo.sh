#!/bin/bash

echo "Starting PX4 Gazebo Simulator..."

# Change to PX4 directory (adjust path as needed)
# Common PX4 installation paths:
if [ -d "$HOME/PX4-Autopilot" ]; then
    cd "$HOME/PX4-Autopilot"
elif [ -d "/opt/px4" ]; then
    cd "/opt/px4"
else
    echo "Error: PX4-Autopilot directory not found!"
    echo "Please update the path in this script to point to your PX4 installation."
    exit 1
fi

# Start PX4 SITL with Gazebo
make px4_sitl gz_x500

echo "Gazebo simulator stopped."
