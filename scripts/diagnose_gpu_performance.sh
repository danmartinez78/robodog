#!/bin/bash
# Diagnose GPU performance issues for Thor or other GPU-enabled hosts
# Run this when experiencing slow inference speeds to identify GPU problems

set -e

echo "=== GPU Performance Diagnostics ==="
echo ""

# Check if nvidia-smi is available
if ! command -v nvidia-smi &> /dev/null; then
    echo "ERROR: nvidia-smi not found. No NVIDIA GPU or drivers installed."
    exit 1
fi

# 1. Check GPU utilization during idle
echo "1. GPU Status (Idle):"
nvidia-smi --query-gpu=index,name,temperature.gpu,utilization.gpu,utilization.memory,memory.used,memory.total,power.draw,power.limit --format=csv
echo ""

# 2. Check GPU clocks
echo "2. GPU Clock Speeds:"
nvidia-smi --query-gpu=clocks.current.graphics,clocks.current.memory,clocks.max.graphics,clocks.max.memory --format=csv
echo ""

# 3. Check power mode
echo "3. GPU Power Management:"
nvidia-smi -q -d POWER | grep -E "Power Management|Power Draw|Power Limit|Default Power Limit"
echo ""

# 4. Check persistence mode
echo "4. GPU Persistence Mode:"
nvidia-smi -q | grep "Persistence Mode"
echo ""

# 5. Check throttling reasons
echo "5. GPU Performance State & Throttling:"
nvidia-smi -q -d PERFORMANCE
echo ""

# 6. Check if GPU is in low-power mode (AMD GPUs)
echo "6. GPU Performance Level (AMD):"
if [ -d /sys/class/drm/card0/device ]; then
    cat /sys/class/drm/card*/device/power_dpm_force_performance_level 2>/dev/null || echo "N/A"
else
    echo "N/A (NVIDIA GPU)"
fi
echo ""

# 7. Check CPU governor (affects PCIe)
echo "7. CPU Governor (affects PCIe performance):"
if [ -d /sys/devices/system/cpu/cpu0/cpufreq ]; then
    cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | head -5
else
    echo "N/A (cpufreq not available)"
fi
echo ""

# 8. Check system load
echo "8. System Load:"
uptime
echo ""

# 9. Check thermal throttling
echo "9. Thermal Status:"
nvidia-smi --query-gpu=temperature.gpu,temperature.memory,clocks_throttle_reasons.active --format=csv
echo ""

# 10. Docker container GPU access (if running)
echo "10. Ollama Container GPU Access:"
if docker ps | grep -q ollama; then
    docker inspect ollama 2>/dev/null | grep -A 10 "DeviceRequests" || echo "No GPU device requests configured"
else
    echo "Ollama container not running"
fi
echo ""

# Summary and recommendations
echo "=== Analysis & Recommendations ==="
echo ""

# Check for N/A values (GPU not initialized)
if nvidia-smi --query-gpu=power.draw --format=csv,noheader | grep -q "N/A"; then
    echo "⚠️  WARNING: GPU showing N/A values - not properly initialized"
    echo "    → Try: sudo nvidia-smi -pm 1 (enable persistence mode)"
    echo "    → Try: Reboot the system"
    echo "    → Check: BIOS PCIe settings"
    echo ""
fi

# Check persistence mode
if nvidia-smi -q | grep "Persistence Mode" | grep -q "Disabled"; then
    echo "⚠️  WARNING: GPU persistence mode disabled"
    echo "    → Run: sudo nvidia-smi -pm 1"
    echo "    → This prevents driver unload and improves performance"
    echo ""
fi

# Check for throttling
if nvidia-smi --query-gpu=clocks_throttle_reasons.active --format=csv,noheader | grep -q "Active"; then
    echo "⚠️  WARNING: GPU is being throttled"
    echo "    → Check: nvidia-smi -q -d PERFORMANCE for reasons"
    echo "    → Common causes: thermal, power limit, HW slowdown"
    echo ""
fi

# Check Docker GPU access
if docker ps | grep -q ollama; then
    if ! docker inspect ollama 2>/dev/null | grep -q "DeviceRequests"; then
        echo "❌ ERROR: Ollama container lacks GPU access"
        echo "    → Restart with: docker run --gpus all ..."
        echo "    → Or use: docker stop ollama && ./scripts/setup_ollama_thor.sh"
        echo ""
    fi
fi

echo "✓ Diagnostics complete"
echo ""
echo "Expected GPU inference speeds (for reference):"
echo "  - 7B models:  ~50-80 tok/s"
echo "  - 14B models: ~20-40 tok/s"
echo "  - 32B models: ~4-8 tok/s"
echo "  - 70B models: ~1.5-3 tok/s"
