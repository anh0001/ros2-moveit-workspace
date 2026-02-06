#!/bin/bash
# Safe launcher for MoveIt Setup Assistant with workarounds for container X11 issues

set -e

echo "=== MoveIt Setup Assistant Safe Launcher ==="
echo "This script applies workarounds for Qt5/RViz crashes in containers"
echo ""

# Check X11 connection
if [ -z "$DISPLAY" ]; then
    echo "ERROR: DISPLAY variable is not set!"
    echo "Please run on host: xhost +SI:localuser:\$(id -un)"
    exit 1
fi

if ! xdpyinfo &>/dev/null; then
    echo "ERROR: X11 server is not accessible!"
    echo "Please run on host: xhost +SI:localuser:\$(id -un)"
    echo "Current DISPLAY: $DISPLAY"
    exit 1
fi

echo "✓ X11 connection OK (DISPLAY=$DISPLAY)"

# Apply Qt5/RViz stability environment variables
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM=xcb
export QT_LOGGING_RULES="*.debug=false;qt.qpa.*=false"

# Force software rendering to avoid GPU driver issues
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_ALWAYS_INDIRECT=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3

# Disable DRI3 which can cause crashes
export LIBGL_DRI3_DISABLE=1

# Disable RViz property tree optimizations that cause segfaults
export OGRE_RTT_MODE=Copy

# Qt threading and rendering tweaks
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_OPENGL=software

# Disable all hardware acceleration paths
unset LIBGL_DEBUG

echo "✓ Applied stability environment variables"
echo "  - Software rendering enabled (LIBGL_ALWAYS_SOFTWARE=1)"
echo "  - Indirect rendering enabled (LIBGL_ALWAYS_INDIRECT=1)"
echo "  - DRI3 disabled"
echo "  - Qt SHM disabled"
echo "  - OGRE RTT Copy mode enabled"
echo ""

# Source ROS workspace
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
    echo "✓ Sourced workspace overlay"
else
    source /opt/ros/humble/setup.bash
    echo "⚠ No workspace overlay found, using base ROS install"
fi

echo ""
echo "=== Launching MoveIt Setup Assistant ===" 
echo "Note: Using maximum stability settings (slower graphics)"
echo "RViz property crashes are a known issue in containerized environments"
echo ""

# Clear any stale RViz config
rm -rf ~/.rviz2/* 2>/dev/null || true

# Launch with error handling
exec ros2 launch moveit_setup_assistant setup_assistant.launch.py
