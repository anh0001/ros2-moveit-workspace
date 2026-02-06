#!/bin/bash
# Alternative launcher for MoveIt Setup Assistant - CLI mode
# This bypasses the problematic GUI file browser

set -e

echo "=== MoveIt Setup Assistant - Alternative Launcher ==="
echo "This script pre-specifies the robot to avoid GUI crashes"
echo ""

if [ $# -lt 1 ]; then
    echo "Usage: $0 <path_to_urdf_or_xacro>"
    echo ""
    echo "Example:"
    echo "  $0 /workspace/src/ranger_description/urdf/ranger_mini_complete.urdf.xacro"
    echo ""
    echo "Available robot descriptions in /workspace/src:"
    find /workspace/src -name "*.urdf" -o -name "*.xacro" | grep -v build | head -5
    exit 1
fi

ROBOT_DESCRIPTION="$1"

if [ ! -f "$ROBOT_DESCRIPTION" ]; then
    echo "ERROR: File not found: $ROBOT_DESCRIPTION"
    exit 1
fi

# Check X11 connection
if [ -z "$DISPLAY" ]; then
    echo "ERROR: DISPLAY variable is not set!"
    exit 1
fi

if ! xdpyinfo &>/dev/null; then
    echo "ERROR: X11 server is not accessible!"
    exit 1
fi

echo "✓ Robot description: $ROBOT_DESCRIPTION"
echo "✓ X11 connection OK (DISPLAY=$DISPLAY)"

# Apply ALL stability environment variables
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM=xcb
export QT_LOGGING_RULES="*.debug=false;qt.qpa.*=false"
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_ALWAYS_INDIRECT=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_DRI3_DISABLE=1
export OGRE_RTT_MODE=Copy
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_OPENGL=software

echo "✓ Applied maximum stability settings"

# Source ROS workspace
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
    echo "✓ Sourced workspace overlay"
else
    source /opt/ros/humble/setup.bash
    echo "⚠ Using base ROS install only"
fi

# Clear RViz cache
rm -rf ~/.rviz2/* 2>/dev/null || true

echo ""
echo "=== Launching with pre-loaded robot description ==="
echo ""

# Launch with robot description pre-specified
exec ros2 launch moveit_setup_assistant setup_assistant.launch.py \
    urdf_path:="$ROBOT_DESCRIPTION"
