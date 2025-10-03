# Shadowhound ROS2 Workspace

This is a ROS2 Humble workspace for the Shadowhound project, set up with a complete development container environment.

## Getting Started

### Prerequisites
- Docker
- VS Code with Dev Containers extension

### Development Environment

This workspace uses a devcontainer that provides:
- ROS2 Humble Desktop Full
- Navigation2 stack
- CycloneDX middleware
- Python development tools (black, isort, pylint, mypy)
- C++ development tools
- PCL libraries for point cloud processing

### Quick Start

1. Open this repository in VS Code
2. When prompted, reopen in container (or use Ctrl+Shift+P -> "Dev Containers: Reopen in Container")
3. Wait for the container to build and setup to complete
4. Use the terminal to start developing:

```bash
# Build the workspace
cb

# Source the workspace
source-ws

# Install dependencies for packages in src/
rosdep-install
```

### Useful Aliases

The devcontainer sets up several helpful aliases:
- `cb` - Build with colcon (includes --symlink-install)
- `cbt` - Run colcon tests
- `cbr` - Build and source workspace
- `source-ws` - Source the workspace setup
- `rosdep-install` - Install ROS dependencies

### Environment Variables

- `ROS_DOMAIN_ID=42` - Isolated ROS network
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` - DDS implementation
- `RCUTILS_LOGGING_BUFFERED_STREAM=1` - Improved logging

## Project Structure

```
├── .devcontainer/          # Development container configuration
│   ├── devcontainer.json   # VS Code devcontainer settings
│   ├── Dockerfile          # Container definition
│   └── setup.sh           # Container setup script
├── src/                   # ROS2 source packages
├── build/                 # Build artifacts (auto-generated)
├── install/               # Installation files (auto-generated)
├── log/                   # Build and runtime logs
└── README.md             # This file
```

## Contributing

1. Create your feature packages in the `src/` directory
2. Follow ROS2 naming conventions
3. Use the provided code formatting tools (black, isort for Python; built-in for C++)
4. Test your code with `colcon test`

## Troubleshooting

### Container Issues
- If the container fails to build, check Docker daemon is running
- Clear Docker cache: `docker system prune -a`
- Rebuild container: Ctrl+Shift+P -> "Dev Containers: Rebuild Container"

### ROS Issues
- If packages aren't found: `source-ws`
- If dependencies missing: `rosdep-install`
- Check ROS environment: `printenv | grep ROS`