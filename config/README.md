# ShadowHound Custom Configuration Files

This directory contains custom configuration files that override defaults from the go2_ros2_sdk package.

## nav2_params.yaml

Custom Nav2 configuration with modifications for DIMOS integration.

### Key Changes from go2_ros2_sdk Default:

1. **`always_send_full_costmap: true`** - Added to both local and global costmaps
   - **Purpose**: Ensures costmaps publish immediately at their configured frequency (1Hz)
   - **Default behavior**: Nav2 only publishes costmap updates when there are changes
   - **Why needed**: DIMOS expects costmap data during initialization, but costmaps may not publish without robot movement
   - **Impact**: Slight increase in bandwidth, but ensures reliable costmap availability

### Configuration Details:

**Local Costmap:**
- Update frequency: 1.0 Hz
- Publish frequency: 1.0 Hz
- Rolling window: 6m x 6m
- Resolution: 0.05m (5cm grid)
- **Always send full costmap: true** ✨

**Global Costmap:**
- Update frequency: 1.0 Hz  
- Publish frequency: 1.0 Hz
- Static map: 500m x 500m
- Resolution: 0.05m (5cm grid)
- **Always send full costmap: true** ✨

### Usage:

The custom config is automatically loaded by `launch/go2_sdk/robot.launch.py` when it exists. No manual configuration needed.

To revert to default behavior:
```bash
rm config/nav2_params.yaml
```

### References:

- Original config: `src/dimos-unitree/.../go2_ros2_sdk/go2_robot_sdk/config/nav2_params.yaml`
- Nav2 documentation: https://navigation.ros.org/configuration/index.html
- Costmap2D parameters: https://navigation.ros.org/configuration/packages/costmap-plugins/index.html
