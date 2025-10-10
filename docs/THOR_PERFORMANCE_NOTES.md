# Thor GPU Performance Notes

## Container Information

**Correct Container for Thor (Jetson AGX Orin):**
```bash
ghcr.io/nvidia-ai-iot/ollama:r38.2.arm64-sbsa-cu130-24.04
```

This is the **Jetson-optimized** container from NVIDIA. Do NOT use standard `ollama/ollama` on Thor - it will be significantly slower.

**Performance difference:**
- Jetson container: 30-40 tok/s (gpt-oss:20b)
- Standard container: 5-10 tok/s (same model)

## GPU Monitoring with jtop

### Installation

Thor requires special patches for `jtop` (Jetson stats monitoring tool):

```bash
# On Thor
cd /path/to/shadowhound
sudo ./scripts/install_jtop_thor.sh
```

This script:
- Installs jtop in a root-owned venv at `/opt/jtop`
- Applies Thor-specific patches (tegra264, CUDA 13.0, module mappings)
- Installs as systemd service for background monitoring
- Provides `sudo jtop` command for interactive monitoring

### Usage

**Interactive monitoring:**
```bash
sudo jtop
```

Provides real-time:
- GPU utilization %
- GPU memory usage (MiB/GiB)
- Power consumption (W)
- Temperature (°C)
- CPU/RAM stats

**Service status:**
```bash
systemctl status jtop.service
journalctl -u jtop --no-pager -e
```

### Why jtop?

Standard `nvidia-smi` on Thor returns `N/A` for many metrics:
- Memory usage: Not Supported
- Power draw: N/A
- Temperature: N/A

`jtop` reads directly from Jetson hardware interfaces and provides accurate metrics.

### For Benchmarking

Run `sudo jtop` in another terminal while benchmarking to monitor:
- Real-time GPU memory allocation per model
- GPU utilization during inference
- Power consumption patterns
- Thermal throttling indicators

## Known Performance Issue: Model Unload Degradation

### Symptoms
After running benchmark script or unloading/reloading models multiple times, GPU performance degrades significantly:

**Example (gpt-oss:20b):**
- Fresh container: **37.00 tok/s** ✅
- After unload/reload cycles: **5.23 tok/s** ❌ (86% slower!)

**Example (phi4:14b):**
- Fresh test: **19.8 tok/s** ✅
- After benchmark operations: **10.60 tok/s** ❌ (47% slower!)

### Root Cause
Unknown - possibly related to:
1. GPU context not being properly restored after model unload
2. Issue specific to Jetson-optimized container
3. CUDA context persistence issue
4. GPU memory fragmentation

### Workaround
**Reboot Thor** to restore full GPU performance. This is required:
- Before running production benchmarks
- When speeds drop significantly during testing
- After multiple model unload/reload cycles

### Testing for This Issue
Run the same model test multiple times:

```bash
# Test 1 - should be fast
docker exec -it ollama ollama run --verbose gpt-oss:20b

# Unload
docker exec -it ollama ollama stop gpt-oss:20b

# Test 2 - check if still fast
docker exec -it ollama ollama run --verbose gpt-oss:20b
```

If Test 2 is significantly slower (>20% drop), the issue is present.

### Best Practices

1. **Reboot before benchmarking**: Ensure clean GPU state
   ```bash
   sudo reboot
   # Wait for Thor to come back
   ./scripts/setup_ollama_thor.sh
   ./scripts/benchmark_ollama_models.sh
   ```

2. **Avoid unnecessary unloads**: Keep models loaded when possible
   - Use `keep_alive` parameter to keep models in memory
   - Don't cycle through models repeatedly

3. **Monitor for degradation**: Check speeds periodically
   ```bash
   # Quick speed check
   time docker exec ollama ollama run gpt-oss:20b "Say hello"
   ```

4. **Restart container as last resort** (may lose GPU access):
   ```bash
   docker restart ollama
   # Or better: stop and recreate
   docker stop ollama && docker rm ollama
   ./scripts/setup_ollama_thor.sh
   ```

## GPU Status Checking

Always verify GPU is working properly:

```bash
# Check GPU is visible and active
nvidia-smi

# Should show proper values (not [N/A]):
# - Power draw
# - Temperature
# - Clock speeds
# - Memory usage

# Check container has GPU access
docker inspect ollama | grep -A 5 DeviceRequests
```

## Benchmark Script Behavior

The `benchmark_ollama_models.sh` script:
- Uses `OLLAMA_HOST` environment variable (defaults to localhost:11434)
- Does NOT manage which container is running
- Assumes correct container is already started via `setup_ollama_thor.sh`
- **May trigger performance degradation** due to model unload/reload cycles

**Recommendation**: Run benchmark immediately after Thor reboot for accurate results.

## Historical Performance Data

### Clean State (Post-Reboot)
- gpt-oss:20b: 37.00 tok/s (eval), 109.63 tok/s (prompt)
- phi4:14b: 19.8 tok/s (expected)

### Degraded State (After Benchmark)
- gpt-oss:20b: 5.23 tok/s (eval), 9.52 tok/s (prompt)
- phi4:14b: 10.60 tok/s
- deepseek-r1:7b: 10.33 tok/s

Performance drops by 50-86% when degraded.

## Investigation TODO

- [ ] Test if issue occurs with standard ollama/ollama container
- [ ] Check if CUDA context persistence mode helps
- [ ] Monitor GPU memory fragmentation over time
- [ ] Test if `nvidia-smi -pm 1` (persistence mode) prevents degradation
- [ ] Check Jetson container release notes for known issues
- [ ] Test with different CUDA versions

## Related Files

- `scripts/setup_ollama_thor.sh` - Proper Thor container setup
- `scripts/benchmark_ollama_models.sh` - May trigger degradation
- `scripts/diagnose_gpu_performance.sh` - GPU health checking
- `docs/OLLAMA_BENCHMARK_RESULTS.md` - Historical benchmark data

---

**Last Updated**: 2025-10-10  
**Issue Status**: Open - workaround available (reboot)
