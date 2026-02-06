# Quick Reference Card

## 🚀 Getting Started (5 Steps)

```bash
# 1. Add your robot packages
cd ~/codes/ros2-moveit-workspace/src/
cp -r /path/to/your_robot_description .

# 2. Enable X11
xhost +local:docker

# 3. Open in VSCode
cd ~/codes/ros2-moveit-workspace
code .
# Press F1 → "Dev Containers: Reopen in Container"

# 4. Build workspace (inside container)
cd /workspace
./scripts/setup.sh  # or manually:
# colcon build --symlink-install
# source install/setup.bash

# 5. Launch Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## 📦 Common Commands (Inside Container)

### Building
```bash
# Build all packages
cb  # alias for: cd /workspace && colcon build --symlink-install

# Build specific package
cbs package_name  # alias for: colcon build --symlink-install --packages-select

# Clean build
rm -rf build install log
colcon build --symlink-install

# Source workspace
setup  # alias for: source /workspace/install/setup.bash
```

### Setup Assistant
```bash
# Launch Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py

# After generating config, build it
colcon build --symlink-install --packages-select your_robot_moveit

# Test with demo
ros2 launch your_robot_moveit demo.launch.py
```

## 🐳 Docker Commands (Without VSCode)

```bash
# Build container
cd ~/codes/ros2-moveit-workspace/.devcontainer
docker-compose build

# Start container
xhost +local:docker
docker-compose up -d

# Attach to container
docker exec -it ros2-moveit-workspace bash

# Stop container
docker-compose down
```

## 🔧 Setup Assistant Steps

1. **Load URDF**: `/workspace/src/your_robot_description/urdf/robot.urdf.xacro`
2. **Self-Collisions**: Generate with 10000 samples
3. **Virtual Joints**: `base_link` → `world` (fixed)
4. **Planning Groups**:
   - Arm: Add kinematic chain or joints
   - Gripper: Add joints (no solver)
5. **Robot Poses**: Define home, stowed, ready, open, closed
6. **End Effectors**: Link gripper to arm
7. **Passive Joints**: Mark wheels, casters
8. **Controllers**: Auto-add FollowJointTrajectory
9. **Author Info**: Your name/email
10. **Generate**: Save to `/workspace/src/your_robot_moveit`

## 🐛 Troubleshooting

```bash
# X11 not working
xhost +local:docker  # On host
echo $DISPLAY        # Should show :0 or :1
xeyes                # Test window

# Package not found
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Stale CMake cache
rm -rf build/package_name install/package_name
colcon build --packages-select package_name

# GPU not working
nvidia-smi  # Should show GPU info
```

## 📂 File Paths

| Location | Path Inside Container | Path on Host |
|----------|----------------------|--------------|
| Workspace | `/workspace` | `~/codes/ros2-moveit-workspace` |
| Source packages | `/workspace/src` | `~/codes/ros2-moveit-workspace/src` |
| Generated config | `/workspace/src/*_moveit` | `~/codes/ros2-moveit-workspace/src/*_moveit` |
| Build artifacts | `/workspace/build` | `~/codes/ros2-moveit-workspace/build` |

## 🔗 Useful Links

- [Full Documentation](README.md)
- [Devcontainer Guide](.devcontainer/README.md)
- [MoveIt Docs](https://moveit.picknik.ai/humble/)
- [Setup Assistant Tutorial](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 💡 Tips

- Always `source install/setup.bash` after building
- Use `--symlink-install` to avoid rebuilds for config changes
- Clean builds when switching between container and host
- Test with `demo.launch.py` before connecting hardware
- Commit both robot description and generated MoveIt config to git
