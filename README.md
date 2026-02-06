# ROS 2 MoveIt Workspace

A generic, containerized workspace for creating MoveIt configurations for any robot using the MoveIt Setup Assistant. Works with any ROS 2 Humble-compatible robot description.

## 🚀 Quick Start

1. **Add your robot packages** to `src/`:
   ```bash
   cd src/
   # Copy, clone, or symlink your robot description packages
   cp -r /path/to/your_robot_description .
   ```

2. **Open in VSCode**:
   ```bash
   xhost +local:docker  # Enable X11
   ls -l ~/.Xauthority  # Required when DISPLAY is localhost:10.0 via SSH
   # If using SSH X11 forwarding, prefer trusted forwarding:
   # ssh -Y <user>@<docker-host>
   code .               # Open in VSCode
   # Press F1 → "Dev Containers: Reopen in Container"
   ```

3. **Build and launch Setup Assistant**:
   ```bash
   # Inside container
   cd /workspace
   colcon build --symlink-install
   source install/setup.bash
   glx-check
   # If DISPLAY is localhost:10.0 and rviz2 fails, use:
   # rviz2-safe
   ros2 launch moveit_setup_assistant setup_assistant.launch.py
   ```

4. **Follow the Setup Assistant GUI** to create your MoveIt configuration

5. **Test the generated config**:
   ```bash
   ros2 launch your_robot_moveit demo.launch.py
   ```

## 📖 Documentation

- **[Devcontainer Setup Guide](.devcontainer/README.md)** - Complete instructions for using the development container
- **[MoveIt Setup Assistant Tutorial](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)** - Official MoveIt documentation

## 🎯 What This Workspace Provides

- ✅ Pre-configured development container with MoveIt 2 and all dependencies
- ✅ X11 forwarding for GUI applications (Setup Assistant, RViz)
- ✅ NVIDIA GPU support for visualization
- ✅ Works with any ROS 2 Humble robot description
- ✅ VSCode integration with ROS extensions
- ✅ Isolated environment (doesn't affect your host ROS installation)

## 📁 Workspace Structure

```
ros2-moveit-workspace/
├── .devcontainer/          # Development container configuration
│   ├── Dockerfile          # Container image definition
│   ├── devcontainer.json   # VSCode devcontainer config
│   ├── docker-compose.yml  # Docker Compose setup
│   └── README.md           # Detailed usage instructions
├── src/                    # Place your robot packages here
│   ├── your_robot_description/
│   ├── your_robot_moveit/  # Generated MoveIt config (after setup)
│   └── ...
├── scripts/                # Helper scripts
│   └── setup.sh            # Quick setup script
├── build/                  # Build artifacts (gitignored)
├── install/                # Install space (gitignored)
├── log/                    # Build logs (gitignored)
└── README.md               # This file
```

## 🛠️ Supported Use Cases

This workspace is designed for:

- **Creating new MoveIt configurations** for custom robots
- **Testing robot URDFs** before deploying to hardware
- **Educational purposes** - learning MoveIt without installing on host
- **Cross-platform development** - works on any system with Docker
- **ARM/Jetson development** - run Setup Assistant on x86_64 for ARM robots

## 🤖 Example Robots

Works with any robot, including:

- Industrial arms (UR, KUKA, ABB, etc.)
- Collaborative robots (Franka Emika, Universal Robots, etc.)
- Mobile manipulators (TurtleBot with arm, custom platforms)
- Custom robot designs
- Multi-arm systems

## 📋 Requirements

- Docker (with NVIDIA runtime for GPU support)
- VSCode with Dev Containers extension (recommended)
- X11 server (for GUI applications)
- Robot URDF/xacro description packages

## 💡 Tips

1. **Keep your robot packages under version control** - the `src/` directory is for your code
2. **Generated MoveIt configs** should also be committed to git
3. **Clean builds when switching** between container and host
4. **Use the demo launch file** to test before connecting hardware
5. **Refer to devcontainer README** for troubleshooting

## 🔗 Resources

- [MoveIt 2 Documentation](https://moveit.picknik.ai/humble/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [VSCode Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)
- [Docker Documentation](https://docs.docker.com/)

## 📝 License

This workspace configuration is provided as-is for use with ROS 2 and MoveIt 2.

## 🤝 Contributing

Feel free to adapt this workspace for your needs. If you find improvements, consider sharing them!

---

**Need help?** Check the [detailed devcontainer documentation](.devcontainer/README.md) or refer to the [MoveIt community forums](https://github.com/moveit/moveit2/discussions).
