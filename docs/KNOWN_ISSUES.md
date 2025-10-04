# Known Issues and Workarounds

## Build and Dependency Issues

### 1. Rosdep Errors for DIMOS Packages ✅ HARMLESS

**Error**:
```
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
speech_processor: Cannot locate rosdep definition for [ament_python]
lidar_processor: Cannot locate rosdep definition for [ament_python]
```

**Cause**: Some DIMOS packages have incorrect `package.xml` format (Python packages with cmake dependencies).

**Impact**: None - these packages are not built by ShadowHound.

**Action**: Ignore this error. ShadowHound packages build and run correctly.

---

### 2. Deformable-DETR CUDA Error ✅ EXPECTED

**Error**:
```
NotImplementedError: Cuda is not availabel
Location: dimos/models/Detic/third_party/Deformable-DETR/models/ops
```

**Cause**: Perception model requires CUDA GPU, devcontainer is CPU-only.

**Impact**: Prevents building DIMOS perception packages, but ShadowHound works fine.

**Workaround**: Build only ShadowHound packages:
```bash
colcon build --symlink-install --packages-select shadowhound_mission_agent shadowhound_bringup
```

**Future**: If perception is needed, skip with COLCON_IGNORE:
```bash
touch src/dimos-unitree/dimos/models/Detic/third_party/Deformable-DETR/COLCON_IGNORE
```

---

### 3. TorchScript Mask R-CNN Build Error ✅ EXPECTED

**Error**:
```
Could not find a package configuration file provided by "Torch"
```

**Cause**: CMake package needs LibTorch (C++ library), we only have Python PyTorch.

**Impact**: Prevents building perception model, but ShadowHound works fine.

**Action**: Not needed for basic operation, skip this package.

---

## Runtime Issues

### 4. "DIMOS not available" Import Error

**Error**:
```
ImportError: cannot import name 'OpenAIAgent'
```

**Cause**: DIMOS not in Python path.

**Solution**:
```bash
export PYTHONPATH="${HOME}/shadowhound/src/dimos-unitree:${PYTHONPATH}"
source install/setup.bash
```

Or add to `~/.bashrc`:
```bash
export PYTHONPATH="${HOME}/shadowhound/src/dimos-unitree:${PYTHONPATH}"
```

---

### 5. Git Shows dimos-unitree Modified

**Symptom**: After `vcs import`, git shows `src/dimos-unitree` as modified.

**Cause**: Wrong commit checked out (before submodule was configured).

**Solution**:
```bash
cd src/dimos-unitree
git checkout 3b0122eec621d32f31d0c32e6565f4614e7de1f9
git submodule update --init --recursive
cd ../..
git status  # Should be clean now
```

**Prevention**: `shadowhound.repos` is already pinned to correct commit (as of Oct 4, 2025).

---

## Robot Connection Issues

### 6. Cannot Connect to Robot

**Symptoms**:
- `ping 192.168.12.1` fails
- No ROS topics visible
- Timeout errors

**Checklist**:
1. Robot is powered on (green LED)
2. Laptop connected to Go2 WiFi (not your home WiFi!)
3. Go2 IP is correct (check with `ip route`)
4. Firewall not blocking (try `sudo ufw disable` temporarily)

**Debug**:
```bash
# Check network interface
ip addr show

# Check routes
ip route

# Test connectivity
ping 192.168.12.1

# Check ROS topics
ros2 topic list
```

---

### 7. Agent Fails to Initialize

**Error**:
```
Failed to initialize agent: <various errors>
```

**Common Causes**:

**Missing API Key**:
```bash
export OPENAI_API_KEY="sk-your-key-here"
```

**Robot not accessible** (when mock_robot:=false):
- Check robot connection (see #6)
- Try with mock_robot:=true first

**Skill library fails**:
- Ensure DIMOS is in PYTHONPATH
- Check dimos-unitree submodules initialized

---

## Expected Behavior

### What's Normal

✅ **Rosdep warnings** about speech_processor and lidar_processor  
✅ **CUDA errors** during full workspace build  
✅ **Detached HEAD** state in dimos-unitree submodule  
✅ **Mock robot warnings** when testing without hardware  

### What's Not Normal

❌ Build fails for shadowhound_mission_agent  
❌ Launch file not found  
❌ Import errors for rclpy or std_msgs  
❌ Python syntax errors  

If you see these, check:
1. ROS2 Humble installed correctly
2. Workspace sourced: `source install/setup.bash`
3. Dependencies installed: `rosdep install --from-paths src --ignore-src -r -y`

---

## Getting Help

1. **Check this file first** for known issues
2. **Review documentation**:
   - `docs/LAPTOP_SETUP.md` - Setup guide
   - `docs/INTEGRATION_STATUS.md` - Architecture
   - Package READMEs in `src/shadowhound_*/`
3. **Check logs**:
   - ROS logs: `~/.ros/log/`
   - Build logs: `/tmp/colcon_build.log`
4. **Verify environment**:
   ```bash
   printenv | grep -E "ROS|OPENAI|PYTHONPATH"
   ```

---

## Contributing

Found a new issue? Update this file with:
- Clear description of the problem
- Root cause (if known)
- Solution or workaround
- Impact level (harmless, warning, critical)
