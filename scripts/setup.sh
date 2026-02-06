#!/bin/bash
# Helper script for setting up the MoveIt workspace

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS 2 MoveIt Workspace Setup ===${NC}"
echo ""

# Check if running inside container
if [ -f "/.dockerenv" ]; then
    CONTAINER=true
    WORKSPACE="/workspace"
    echo -e "${GREEN}✓ Running inside container${NC}"
else
    CONTAINER=false
    WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    echo -e "${YELLOW}⚠ Running on host machine${NC}"
fi

cd "$WORKSPACE"

# Step 1: Check for packages in src/
echo ""
echo -e "${YELLOW}Step 1: Checking for robot packages...${NC}"
if [ -z "$(ls -A src/ 2>/dev/null | grep -v '.gitkeep' | grep -v 'README')" ]; then
    echo -e "${RED}✗ No packages found in src/${NC}"
    echo ""
    echo "Please add your robot description packages to src/ directory:"
    echo "  cd src/"
    echo "  cp -r /path/to/your_robot_description ."
    echo "  # or"
    echo "  git clone https://github.com/your-org/your_robot_description.git"
    echo ""
    exit 1
else
    echo -e "${GREEN}✓ Found packages in src/${NC}"
    ls -1 src/ | grep -v '.gitkeep' | grep -v 'README' | sed 's/^/  - /'
fi

# Step 2: Source ROS 2
echo ""
echo -e "${YELLOW}Step 2: Sourcing ROS 2 Humble...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ ROS 2 Humble sourced${NC}"
else
    echo -e "${RED}✗ ROS 2 Humble not found${NC}"
    exit 1
fi

# Step 3: Clean build (optional)
echo ""
read -p "Clean previous build artifacts? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Cleaning build, install, log directories...${NC}"
    rm -rf build install log
    echo -e "${GREEN}✓ Cleaned${NC}"
fi

# Step 4: Update package lists (if in container)
if [ "$CONTAINER" = true ]; then
    echo ""
    echo -e "${YELLOW}Step 3: Updating package lists...${NC}"
    sudo apt-get update > /dev/null 2>&1
    echo -e "${GREEN}✓ Package lists updated${NC}"
fi

# Step 5: Install dependencies
echo ""
echo -e "${YELLOW}Step 4: Installing dependencies with rosdep...${NC}"
rosdep update > /dev/null 2>&1
rosdep install --from-paths src --ignore-src -r -y
echo -e "${GREEN}✓ Dependencies installed${NC}"

# Step 6: Build workspace
echo ""
echo -e "${YELLOW}Step 5: Building workspace...${NC}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

# Step 7: Source workspace
echo ""
echo -e "${YELLOW}Step 6: Sourcing workspace...${NC}"
source install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"

# Done
echo ""
echo -e "${GREEN}=== Setup Complete! ===${NC}"
echo ""
echo "Next steps:"
echo "  1. Source the workspace:"
echo "     source install/setup.bash"
echo ""
echo "  2. Launch MoveIt Setup Assistant:"
echo "     ros2 launch moveit_setup_assistant setup_assistant.launch.py"
echo ""
echo "  3. After generating your MoveIt config, build it:"
echo "     colcon build --symlink-install --packages-select your_robot_moveit"
echo ""
echo "  4. Test with demo mode:"
echo "     ros2 launch your_robot_moveit demo.launch.py"
echo ""
