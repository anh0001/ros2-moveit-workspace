# ROS 2 Humble MoveIt Development Container

A generic, reusable development container for creating MoveIt configurations for any robot using the MoveIt Setup Assistant.

## Features

- **Base Image**: Official `moveit/moveit2:humble-release`
- **ROS 2 Humble**: Full installation with all core packages
- **MoveIt 2**: Complete stack including Setup Assistant
- **X11 Forwarding**: GUI support for RViz and Setup Assistant
- **GPU Acceleration**: NVIDIA GPU support for visualization
- **Development Tools**: git, cmake, colcon, rosdep, VSCode extensions

## Prerequisites

1. **Docker** installed and running
2. **NVIDIA Docker Runtime** (for GPU support):
   ```bash
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   sudo apt-get update && sudo apt-get install -y nvidia-docker2
   sudo systemctl restart docker
   ```

3. **VSCode** with "Dev Containers" extension (recommended)
4. **X11 server** configured

## Quick Start

### Step 1: Prepare Your Robot Packages

Place your robot description packages in the `src/` directory:

```bash
cd ~/codes/ros2-moveit-workspace/src/

# Option A: Copy from existing workspace
cp -r /path/to/your_robot_description .

# Option B: Clone from repository
git clone https://github.com/your-org/your_robot_description.git

# Option C: Create symbolic links
ln -s /path/to/your_robot_description .
```

**Example for multiple packages:**
```
src/
├── your_robot_description/
├── your_robot_arm_description/
├── your_gripper_description/
└── any_dependencies/
```

### Step 2: Enable X11 Forwarding

On your host machine:
```bash
xhost +local:docker
```

### Step 3: Open in VSCode (Recommended)

1. Open the `ros2-moveit-workspace` folder in VSCode
2. Press `F1` → `Dev Containers: Reopen in Container`
3. Wait for the container to build (5-10 minutes first time)

### Step 4: Build Your Robot Packages

Inside the container terminal:
```bash
cd /workspace
source /opt/ros/humble/setup.bash

# Clean any stale build artifacts (important!)
rm -rf build install log

# Update package lists
sudo apt-get update

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build your robot packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Step 5: Launch MoveIt Setup Assistant

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## MoveIt Setup Assistant Workflow

### 1. Load Your Robot URDF

- Click **"Create New MoveIt Configuration Package"**
- Browse to your robot's main URDF/xacro file:
  - Path: `/workspace/src/your_robot_description/urdf/robot.urdf.xacro`
- Click **"Load Files"**
- Wait for the robot model to appear

### 2. Generate Self-Collision Matrix

- Click **"Self-Collisions"**
- Set sampling density: `10000`
- Click **"Generate Collision Matrix"**
- Review disabled collision pairs

### 3. Define Virtual Joints

- Click **"Virtual Joints"** → **"Add Virtual Joint"**
- Configure:
  - **Name**: `virtual_joint`
  - **Child Link**: Your robot's base link (e.g., `base_footprint`, `base_link`)
  - **Parent Frame**: `world` or `map`
  - **Joint Type**: `fixed` (for stationary robots) or `planar`/`floating` (for mobile robots)

### 4. Create Planning Groups

Create groups for each manipulator/arm:

**Arm Group:**
- **Group Name**: e.g., `manipulator`, `arm`, `left_arm`
- **Kinematic Solver**: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- **Add Joints** or **Add Kin. Chain**:
  - For chain: Base link → Tip link
  - For joints: Select all arm joints

**Gripper/End-Effector Group:**
- **Group Name**: e.g., `gripper`, `hand`
- **Kinematic Solver**: `None`
- **Add Joints**: Select gripper joints

### 5. Define Robot Poses

Add useful preset configurations:

**Examples:**
- `home`: All joints at zero or safe home position
- `stowed`: Compact pose for navigation/storage
- `ready`: Ready-to-manipulate position
- `open` / `closed`: Gripper states

### 6. Configure End Effectors

- Click **"End Effectors"** → **"Add End Effector"**
- Link gripper group to arm group:
  - **End Effector Name**: e.g., `gripper`
  - **End Effector Group**: Your gripper group
  - **Parent Link**: Wrist/last arm link
  - **Parent Group**: Your arm group

### 7. Mark Passive Joints (if applicable)

For mobile robots or systems with passive elements:
- Click **"Passive Joints"**
- Add joints that should be ignored during planning:
  - Wheel joints
  - Caster joints
  - Passive mechanical elements

### 8. Configure ROS 2 Controllers

- Click **"ROS 2 Controllers"**
- Click **"Auto Add FollowJointsTrajectory Controllers"**
- Review and adjust controller names to match your hardware

### 9. Author Information

- Click **"Author Information"**
- Fill in your name and email

### 10. Generate Configuration Package

- Click **"Configuration Files"**
- Set paths:
  - **Save Path**: `/workspace/src/`
  - **Package Name**: `your_robot_moveit` (e.g., `ranger_piper_moveit`)
- Click **"Generate Package"**
- Wait for completion
- Click **"Exit Setup Assistant"**

## After Generation

### Build the MoveIt Config Package

**Inside the container:**
```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select your_robot_moveit
source install/setup.bash
```

**Or on your host machine (after exiting container):**
```bash
cd ~/codes/ros2-moveit-workspace
colcon build --symlink-install --packages-select your_robot_moveit
source install/setup.bash
```

### Test with Demo Mode

```bash
ros2 launch your_robot_moveit demo.launch.py
```

This launches RViz with fake controllers for testing.

## Alternative: Without VSCode

### Using Docker Compose

```bash
cd ~/codes/ros2-moveit-workspace/.devcontainer

# Enable X11
xhost +local:docker

# Build and start container
docker-compose build
docker-compose up -d

# Attach to container
docker exec -it ros2-moveit-workspace bash

# Inside container, follow Steps 4-5 from Quick Start
```

### Stop container:
```bash
docker-compose down
```

## Troubleshooting

### X11 Display Issues

```bash
# On host
xhost +local:docker
echo $DISPLAY  # Should show something like :0 or :1

# Inside container
echo $DISPLAY  # Should match host
xeyes          # Test window should appear
```

### GPU Not Working

Test NVIDIA runtime:
```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### Package Not Found

Make sure packages are built and sourced:
```bash
cd /workspace
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Build Errors - Stale Cache

Clean CMake cache:
```bash
cd /workspace
rm -rf build/your_package install/your_package
colcon build --packages-select your_package --symlink-install
```

### Setup Assistant Crashes

- Verify you're using the devcontainer (not running on Jetson directly)
- Check X11 forwarding is working (`xeyes` test)
- Ensure GPU drivers are working (`nvidia-smi` inside container)

## File Structure

```
ros2-moveit-workspace/
├── .devcontainer/
│   ├── devcontainer.json       # VSCode devcontainer config
│   ├── Dockerfile              # Container image definition
│   ├── docker-compose.yml      # Docker Compose config
│   └── README.md               # This file
├── src/
│   ├── your_robot_description/ # Your robot packages
│   └── your_robot_moveit/      # Generated MoveIt config (after setup)
├── build/                      # Build artifacts (gitignored)
├── install/                    # Installed packages (gitignored)
├── log/                        # Build logs (gitignored)
└── README.md                   # Main repository README
```

## Tips & Best Practices

1. **Always clean build artifacts** when switching between container and host builds
2. **Use symlink-install** to avoid rebuilding for Python/config changes
3. **Source the workspace** before launching any ROS commands
4. **Save your work frequently** - container changes to files persist on host
5. **Use version control** for your robot packages and generated configs
6. **Test in demo mode first** before connecting to real hardware

## Useful Aliases (already configured)

Inside the container:
```bash
cb      # cd /workspace && colcon build --symlink-install
cbs     # colcon build --symlink-install --packages-select
setup   # source /workspace/install/setup.bash
```

## References

- [MoveIt 2 Documentation](https://moveit.picknik.ai/humble/)
- [Setup Assistant Tutorial](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)
- [VSCode Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)
- [Docker Documentation](https://docs.docker.com/)
