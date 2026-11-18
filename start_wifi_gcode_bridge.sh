#!/usr/bin/env bash
echo "======================================================================"
echo "   DENSO VS-6556 WI-FI G-CODE STREAMING SERVER (ROS 2 -> RASPBERRY PI)"
echo "======================================================================"
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null
python3 "$SCRIPT_DIR/src/denso_gazebo/scripts/denso_wifi_gcode_server.py"

