# Examples: Adding Robots to the Workspace

This file contains examples for adding different robot configurations to the MoveIt workspace.

## Example 1: Ranger Garden Assistant (Mobile Manipulator)

The Ranger Garden Assistant is a mobile platform with a PiPER robotic arm.

### Add Ranger Packages

```bash
cd ~/codes/ros2-moveit-workspace/src/

# Copy Ranger base description
cp -r ~/codes/ranger-garden-assistant/src/ranger_description .

# Copy PiPER arm packages (if needed as dependencies)
cp -r ~/codes/ranger-garden-assistant/src/piper_ros .
```

### Build and Launch

```bash
cd ~/codes/ros2-moveit-workspace

# Open in devcontainer (VSCode)
xhost +local:docker
code .
# F1 → "Dev Containers: Reopen in Container"

# Inside container:
./scripts/setup.sh

# Launch Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

### Setup Assistant Configuration for Ranger

**URDF Path**: `/workspace/src/ranger_description/urdf/ranger_complete.urdf.xacro`

**Virtual Joint**:
- Name: `virtual_joint`
- Child Link: `base_footprint`
- Parent Frame: `map`
- Type: `fixed` (or `planar` for mobile manipulation)

**Planning Groups**:

1. **Arm Group** (`piper_arm`):
   - Kinematic Chain:
     - Base Link: `piper_world`
     - Tip Link: `piper_link_6`
   - Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
   - Includes joints: `piper_joint_1` through `piper_joint_6`

2. **Gripper Group** (`piper_gripper`):
   - Joints: `piper_joint_gripper`
   - Solver: None

**Poses**:
- `home`: All joints at 0.0
- `stowed`: Compact position for navigation
- `open`: Gripper open (0.035)
- `closed`: Gripper closed (0.0)

**End Effector**:
- Name: `piper_gripper`
- Group: `piper_gripper`
- Parent Link: `piper_link_6`
- Parent Group: `piper_arm`

**Passive Joints** (mark these as passive):
- `fl_wheel`, `fr_wheel`, `rl_wheel`, `rr_wheel`
- `fl_steering_joint`, `fr_steering_joint`, `rl_steering_joint`, `rr_steering_joint`

**Save Package**: `/workspace/src/ranger_piper_moveit`

---

## Example 2: Universal Robots UR5

A common 6-DOF industrial arm.

### Add UR5 Packages

```bash
cd ~/codes/ros2-moveit-workspace/src/

# Clone official UR description
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description

# Or if you have a custom description
cp -r /path/to/ur5_description .
```

### Setup Assistant Configuration

**URDF Path**: `/workspace/src/ur_description/urdf/ur5.urdf.xacro`

**Virtual Joint**:
- Name: `world_joint`
- Child Link: `base_link`
- Parent Frame: `world`
- Type: `fixed`

**Planning Group** (`manipulator`):
- Kinematic Chain:
  - Base Link: `base_link`
  - Tip Link: `tool0` or `wrist_3_link`
- Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`

**Poses**:
- `home`: All joints at 0
- `up`: Arm pointing upward
- `ready`: Common working position

**Save Package**: `/workspace/src/ur5_moveit`

---

## Example 3: Franka Emika Panda (with Gripper)

A 7-DOF collaborative robot with integrated gripper.

### Add Panda Packages

```bash
cd ~/codes/ros2-moveit-workspace/src/

# Clone Franka description (if not already available)
git clone -b humble https://github.com/frankaemika/franka_ros2.git
# Or use official franka_description package
```

### Setup Assistant Configuration

**URDF Path**: `/workspace/src/franka_ros2/franka_description/robots/panda_arm.urdf.xacro`

**Virtual Joint**:
- Name: `virtual_joint`
- Child Link: `panda_link0`
- Parent Frame: `world`
- Type: `fixed`

**Planning Groups**:

1. **Arm** (`panda_arm`):
   - Kinematic Chain:
     - Base Link: `panda_link0`
     - Tip Link: `panda_link8`
   - Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`

2. **Gripper** (`hand`):
   - Joints: `panda_finger_joint1`, `panda_finger_joint2`
   - Solver: None

**End Effector**:
- Name: `hand`
- Group: `hand`
- Parent Link: `panda_link8`
- Parent Group: `panda_arm`

**Save Package**: `/workspace/src/panda_moveit`

---

## Example 4: TurtleBot with PhantomX Pincher Arm

A mobile manipulator combining a TurtleBot base with a small arm.

### Add Packages

```bash
cd ~/codes/ros2-moveit-workspace/src/

# Clone TurtleBot description
git clone -b humble https://github.com/turtlebot/turtlebot4.git

# Clone arm description
git clone https://github.com/your-org/phantomx_pincher_description.git

# Create combined URDF if needed
```

### Setup Assistant Configuration

**Planning Groups**:

1. **Arm** (`arm`):
   - All arm joints
   - Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`

2. **Gripper** (`gripper`):
   - Gripper joints
   - Solver: None

**Passive Joints**:
- Mark all wheel joints as passive

**Save Package**: `/workspace/src/turtlebot_arm_moveit`

---

## Example 5: Custom Robot from Scratch

If you're starting from a URDF you created yourself.

### Prepare Your URDF

```bash
cd ~/codes/ros2-moveit-workspace/src/

# Create your package structure
mkdir -p my_robot_description/urdf
mkdir -p my_robot_description/meshes/visual
mkdir -p my_robot_description/meshes/collision
mkdir -p my_robot_description/launch
mkdir -p my_robot_description/config

# Copy your URDF/xacro
cp /path/to/my_robot.urdf.xacro my_robot_description/urdf/

# Copy meshes
cp -r /path/to/meshes/* my_robot_description/meshes/

# Create package.xml and CMakeLists.txt
# (Use templates or copy from another robot package)
```

### Minimal package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_description</name>
  <version>1.0.0</version>
  <description>Description of my custom robot</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Minimal CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### Build and Launch

```bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

---

## Tips for Any Robot

1. **Check URDF Validity** before using Setup Assistant:
   ```bash
   check_urdf your_robot.urdf
   # or for xacro:
   xacro your_robot.urdf.xacro > /tmp/robot.urdf
   check_urdf /tmp/robot.urdf
   ```

2. **Visualize First**:
   ```bash
   ros2 launch your_robot_description display.launch.py
   # Check that all links and joints appear correctly in RViz
   ```

3. **Document Joint Names**: List all joint names before starting:
   ```bash
   grep 'joint name=' your_robot.urdf.xacro
   ```

4. **Test Incrementally**: After generating MoveIt config:
   - Test demo mode first
   - Check collision detection
   - Test IK solver
   - Then connect to real hardware

5. **Keep Notes**: Document your configuration choices:
   - Joint limits
   - Planning group decisions
   - Controller mappings
   - Any special considerations

---

## Need Help?

- Check the [main README](README.md)
- Review [devcontainer documentation](.devcontainer/README.md)
- Consult [MoveIt Setup Assistant tutorial](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)
- Ask on [MoveIt Discourse](https://github.com/moveit/moveit2/discussions)
