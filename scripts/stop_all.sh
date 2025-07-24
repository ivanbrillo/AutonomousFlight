#!/bin/bash

echo "Stopping all components..."

# Kill Gazebo processes
pkill -f gazebo
pkill -f px4

# Kill uORB client
pkill -f micrortps_client

# Kill offboard control
pkill -f offboard_control

echo "All components stopped."
