@echo off
title DENSO VS-6556 WI-FI G-CODE BRIDGE SERVER
echo ======================================================================
echo    DENSO VS-6556 WI-FI G-CODE STREAMING SERVER (ROS 2 -^> RASPBERRY PI)
echo ======================================================================
python "%~dp0src\denso_gazebo\scripts\denso_wifi_gcode_server.py"
pause

