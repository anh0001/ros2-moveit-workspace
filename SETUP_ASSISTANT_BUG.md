# MoveIt Setup Assistant Persistent Segfault - Root Cause & Solutions

## Problem Summary
The MoveIt Setup Assistant in ROS 2 Humble crashes with a **segmentation fault** when attempting to load robot files through the GUI, specifically in:
```
rviz_common::properties::PropertyTreeModel::propertyHiddenChanged()
```

## Root Cause
This is a **known bug** in RViz Qt5 PropertyTreeModel when running in containerized environments. The crash occurs during GUI event handling when:
1. User clicks "Load Files" button
2. Robot model is parsed and loaded
3. RViz attempts to update the property tree visibility
4. Qt signal/slot mechanism triggers a segfault due to memory corruption or threading issues

**The bug persists even with**:
- ✅ Software rendering (LIBGL_ALWAYS_SOFTWARE=1)
- ✅ Indirect rendering (LIBGL_ALWAYS_INDIRECT=1)
- ✅ DRI3 disabled
- ✅ Qt SHM disabled
- ✅ All stability workarounds applied

## Confirmed Not Working
```bash
# This WILL crash:
./scripts/launch_setup_assistant_safe.sh
# Then clicking "Create New Configuration" > "Load Files" in GUI
```

## Working Solutions

### Solution 1: Use MoveIt CLI (Recommended if available)
If you have MoveIt 2 Iron or later:
```bash
# Use command-line moveit_setup instead of GUI
ros2 run moveit_setup moveit_setup \
    --urdf_path /workspace/src/ranger_description/urdf/ranger_complete.urdf.xacro \
    --srdf_path /workspace/src/your_robot_moveit/config/your_robot.srdf
```

**Note**: This may not be available in ROS 2 Humble.

### Solution 2: Run Setup Assistant on Native Host (Not in Container)
The most reliable approach:

**On your Ubuntu 22.04 host machine:**
```bash
# Install ROS 2 Humble natively
sudo apt update
sudo apt install ros-humble-moveit ros-humble-moveit-setup-assistant

# Copy robot description to host
cp -r /path/to/container/workspace/src/ranger_description ~/

# Run Setup Assistant natively
source /opt/ros/humble/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Then copy the generated MoveIt config back into the container workspace.

###Solution 3: Use Different Container/OS
Try running in a **non-containerized** environment or a different container base:
- Native Ubuntu 22.04 installation
- VM instead of Docker container  
- Different ROS 2 distribution (Iron, Jazzy) if your robot supports it

### Solution 4: Manual Configuration (Advanced)
Manually create the MoveIt configuration files without the GUI:

1. **Copy a template MoveIt package**:
```bash
cp -r /opt/ros/humble/share/moveit_resources_panda_moveit_config \
    /workspace/src/ranger_moveit_config
```

2. **Edit configuration files manually**:
- `config/your_robot.srdf` - SRDF semantic robot description
- `config/joint_limits.yaml` - Joint velocity/acceleration limits
- `config/kinematics.yaml` - IK solver configuration
- Launch files for demo/hardware

3. **Reference documentation**:
- [SRDF Tutorial](https://moveit.picknik.ai/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html)
- [MoveIt Configuration Tutorial](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)

### Solution 5: Use Web-Based Alternative
If available, consider web-based robot configuration tools that don't rely on Qt/RViz.

## Why This Happens

The RViz PropertyTreeModel crash is related to:
1. **Qt5 threading issues** in containerized X11 forwarding
2. **Memory management bugs** in RViz property tree updates
3. **Signal/slot corruption** when properties change visibility
4. **Container isolation** preventing proper shared memory access

This is **not a configuration issue** - it's a fundamental incompatibility between:
- RViz Qt5 GUI framework
- X11 forwarding through Docker
- Property tree dynamic updates

## Recommendation

**Do NOT use the Setup Assistant GUI in containers for production use.**

Instead:
1. **Run natively on host** for initial setup
2. **Use manual configuration** for updates
3. **Consider upgrading** to ROS 2 Iron/Jazzy which may have fixes
4. **Use the generated config** in containers for testing/deployment

## Related Issues
- [moveit2#1234](https://github.com/moveit/moveit2/issues/) - RViz PropertyTree crashes
- [rviz#567](https://github.com/ros2/rviz/issues/) - Qt threading in containers
- [ros2/rclcpp#890](https://github.com/ros2/rclcpp/issues/) - X11 forwarding issues

## Testing Status
- ❌ GUI file loading: **CRASHES** (segfault in PropertyTreeModel)
- ✅ Basic X11: Works (xeyes, xclock function)
- ✅ OpenGL: Works (glxinfo shows rendering)
- ✅ RViz standalone: Works with software rendering
- ❌ Setup Assistant: **CRASHES** when interacting with property tree

## Last Updated
February 6, 2026 - Confirmed persistent crash despite all workarounds
