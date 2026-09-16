#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

# Set default DDS and GUI variables
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=${DISPLAY:-:0}
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export LP_NUM_THREADS=2
export MESA_GL_VERSION_OVERRIDE=4.5
export OGRE_RTT_MODE=FBO
export QT_QPA_PLATFORM=xcb
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

exec "$@"
