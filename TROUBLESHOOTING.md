# Troubleshooting Guide

## MoveIt Setup Assistant Segmentation Fault

### Problem
The Setup Assistant crashes with a segmentation fault when trying to load robot files:
```
[moveit_setup_assistant-1] Segmentation fault (Signal sent by the kernel [(nil)])
[ERROR] [moveit_setup_assistant-1]: process has died [pid XXXX, exit code -11]
```

### Root Cause
This is a known issue with RViz/Qt5 in containerized environments. The crash occurs in `rviz_common::properties::PropertyTreeModel::propertyHiddenChanged()` due to:
- Hardware OpenGL rendering conflicts with X11 forwarding
- Qt5 shared memory (MIT-SHM) issues in containers
- DRI3 incompatibilities between host and container graphics stacks

### Solutions (in order of preference)

#### Solution 1: Use the Safe Launcher (Recommended)
```bash
cd /workspace
source install/setup.bash
./scripts/launch_setup_assistant_safe.sh
```

This script automatically applies all necessary workarounds.

#### Solution 2: Manual Environment Variables
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_DRI3_DISABLE=1
export QT_X11_NO_MITSHM=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3
export QT_LOGGING_RULES="*.debug=false;qt.qpa.*=false"

ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

#### Solution 3: Update Container Configuration
If you need a permanent fix, rebuild the container with updated environment defaults:

1. The Dockerfile already includes base settings
2. For additional safety, add to [.devcontainer/devcontainer.json](file://.devcontainer/devcontainer.json):
```json
"containerEnv": {
  "LIBGL_ALWAYS_SOFTWARE": "1",
  "LIBGL_DRI3_DISABLE": "1",
  "GALLIUM_DRIVER": "llvmpipe"
}
```
3. Rebuild: Press F1 → "Dev Containers: Rebuild Container"

### What These Variables Do

- **LIBGL_ALWAYS_SOFTWARE=1**: Forces Mesa to use software rendering (llvmpipe) instead of hardware GPU drivers. Slower but more stable in containers.
- **LIBGL_DRI3_DISABLE=1**: Disables DRI3 (Direct Rendering Infrastructure 3) which can have compatibility issues between host and container.
- **QT_X11_NO_MITSHM=1**: Prevents Qt from using MIT-SHM (shared memory extensions), which don't work well across container boundaries.
- **GALLIUM_DRIVER=llvmpipe**: Explicitly selects the LLVMpipe software renderer.
- **MESA_GL_VERSION_OVERRIDE=3.3**: Ensures Mesa reports a modern OpenGL version.

### Trade-offs

**Software Rendering (LIBGL_ALWAYS_SOFTWARE=1)**
- ✅ Pros: Stable, works in all container configurations
- ❌ Cons: Slower graphics performance, higher CPU usage

**Hardware Rendering (LIBGL_ALWAYS_SOFTWARE=0)**
- ✅ Pros: Fast, GPU-accelerated graphics
- ❌ Cons: May crash with driver mismatches, requires proper NVIDIA runtime setup

### When to Use Hardware Rendering

If your setup is stable and you need performance:
```bash
export LIBGL_ALWAYS_SOFTWARE=0
export LIBGL_ALWAYS_INDIRECT=0
# Keep these for stability:
export LIBGL_DRI3_DISABLE=1
export QT_X11_NO_MITSHM=1

ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## Other Common Issues

### X11 Connection Refused
```
Error: Can't open display: :0
```

**Solution:**
```bash
# On host:
xhost +SI:localuser:$(id -un)

# Verify DISPLAY inside container:
echo $DISPLAY
xeyes  # Test window should appear
```

### No Protocol Specified / Invalid MIT-MAGIC-COOKIE
```
No protocol specified
qt.qpa.xcb: could not connect to display :0
```

**Solution:**
Check X11 authentication:
```bash
# Inside container:
ls -l $XAUTHORITY  # Should exist
xauth list  # Should show entries

# If empty, the container may need rebuilding or X11 forwarding needs setup
```

### RViz/Setup Assistant Freezes

If the GUI becomes unresponsive:
```bash
# Check if process is alive:
ps aux | grep moveit_setup_assistant

# Force quit:
pkill -9 moveit_setup_assistant

# Restart with safe mode:
./scripts/launch_setup_assistant_safe.sh
```

### "Failed to load platform plugin 'xcb'"
```
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
```

**Solution:**
Missing Qt XCB libraries (should be in Dockerfile):
```bash
sudo apt-get update
sudo apt-get install -y \
    libx11-xcb1 \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shm0 \
    libxcb-xinerama0
```

### GPU Not Detected / NVIDIA Driver Issues
```
nvidia-smi: command not found
```

**Solution:**
1. Check NVIDIA runtime is installed on host
2. Verify Docker can access GPU:
```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```
3. If not working, install NVIDIA Container Toolkit (see [.devcontainer/README.md](file://.devcontainer/README.md))

### Build Errors After Container Rebuild
```
CMake Error: The source directory does not exist
```

**Solution:**
Clean stale build artifacts:
```bash
cd /workspace
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

## Helper Commands

```bash
# Check X11 and OpenGL status:
glx-check

# Launch RViz with safe settings:
rviz2-safe

# Launch Setup Assistant with safe settings:
setup-assistant-safe

# Quick rebuild:
cb  # alias for: cd /workspace && colcon build --symlink-install

# Rebuild specific package:
cbs package_name  # alias for: colcon build --symlink-install --packages-select
```

## Getting More Help

1. Check [MoveIt 2 Troubleshooting](https://moveit.picknik.ai/humble/doc/how_to_guides/how_to_guides.html)
2. Review [.devcontainer/README.md](file://.devcontainer/README.md) for container-specific issues
3. Search [MoveIt Discussion Forums](https://github.com/moveit/moveit2/discussions)
4. Check container logs: `docker logs ros2-moveit-workspace`

## Reporting Issues

When reporting issues, include:
```bash
# System info:
echo "DISPLAY: $DISPLAY"
echo "XAUTHORITY: $XAUTHORITY"
glxinfo -B
nvidia-smi  # if applicable

# ROS info:
ros2 doctor
printenv | grep ROS

# Error logs from terminal output
```
