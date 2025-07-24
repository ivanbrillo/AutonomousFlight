#!/bin/bash

# Get the project root directory (parent of scripts folder)
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Starting PX4 Offboard Control System..."
echo "Project directory: $PROJECT_DIR"
echo "Scripts directory: $SCRIPT_DIR"

# Start Gazebo simulator in first terminal
gnome-terminal --title="Gazebo Simulator" -- bash -c "
    echo 'Starting Gazebo Simulator...';
    cd $SCRIPT_DIR;
    ./start_gazebo.sh;
    exec bash
"

# Wait a bit for Gazebo to start
sleep 3

# Start uORB translator client in second terminal
gnome-terminal --title="uORB Translator" -- bash -c "
    echo 'Starting uORB Translator Client...';
    cd $SCRIPT_DIR;
    ./start_uorb_client.sh;
    exec bash
"

# Wait a bit for uORB client to connect
sleep 2

# Build and start the project in third terminal
gnome-terminal --title="Offboard Control" -- bash -c "
    echo 'Building and starting Offboard Control...';
    cd $SCRIPT_DIR;
    ./start_project.sh;
    exec bash
"

echo "All components started in separate terminals."
echo "Check each terminal window for status."
