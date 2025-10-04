# Environment Configuration Guide

This directory contains environment configuration files for ShadowHound.

## Files

| File | Purpose | When to Use |
|------|---------|-------------|
| `.env.example` | Master reference with ALL variables | Documentation, copy to start |
| `.env.development` | Development setup (mock robot) | Local development |
| `.env.production` | Production setup (real robot) | Deployment |
| `.env` | **Your actual config** (gitignored) | Always (created by you) |

## Quick Start

### For Development (No Robot Hardware)

```bash
# Copy development template
cp .env.development .env

# Edit with your OpenAI API key
nano .env  # or vim, vscode, etc.

# Add your key:
OPENAI_API_KEY=sk-proj-your-actual-key-here

# Done! Launch:
ros2 launch shadowhound_mission_agent bringup.launch.py
```

**Open**: http://localhost:8080

### For Production (Real Robot)

```bash
# Copy production template
cp .env.production .env

# Edit with your settings
nano .env

# Add your OpenAI key and robot IP:
OPENAI_API_KEY=sk-proj-your-actual-key-here
GO2_IP=192.168.1.103  # Your robot's IP

# Secure the file
chmod 600 .env

# Launch
ros2 launch shadowhound_mission_agent bringup.launch.py
```

## Essential Variables

### Minimum Required

```bash
# Just this one for mock robot development:
OPENAI_API_KEY=sk-...

# Add this for real robot:
GO2_IP=192.168.1.103
MOCK_ROBOT=false
```

### Recommended

```bash
# Avoid ROS2 network conflicts
ROS_DOMAIN_ID=42

# Better performance
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Control web interface
ENABLE_WEB_INTERFACE=true
WEB_PORT=8080
```

## Variable Categories

### 🔐 OpenAI / LLM
- `OPENAI_API_KEY` ⚠️ **REQUIRED** - Your OpenAI API key
- `OPENAI_MODEL` - Which ChatGPT model (gpt-4o, gpt-4, gpt-3.5-turbo)
- `OPENAI_EMBEDDING_MODEL` - Embeddings for RAG
- `AGENT_BACKEND` - "cloud" or "local"

### 🤖 Robot
- `GO2_IP` ⚠️ **REQUIRED for real robot** - Robot's IP address
- `GO2_INTERFACE` - "ethernet", "wifi", or "webrtc"
- `MOCK_ROBOT` - Use mock robot (true/false)

### 📡 ROS2
- `ROS_DOMAIN_ID` - Network isolation (0-101)
- `RMW_IMPLEMENTATION` - DDS implementation
- `RCUTILS_LOGGING_LEVEL` - Log verbosity

### 🌐 Web Interface
- `ENABLE_WEB_INTERFACE` - Enable dashboard (true/false)
- `WEB_PORT` - HTTP port (default: 8080)
- `WEB_HOST` - Bind address (0.0.0.0 or 127.0.0.1)

### 🧠 RAG / Memory
- `USE_LOCAL_EMBEDDINGS` - Free local embeddings (true/false)
- `RAG_QUERY_N` - How many docs to retrieve (1-20)
- `RAG_SIMILARITY_THRESHOLD` - Match strictness (0.0-1.0)

### 📊 Logging
- `LOG_LEVEL` - Python log level (DEBUG, INFO, WARNING, ERROR)
- `DIMOS_DEBUG` - Verbose DIMOS logging (true/false)

## Common Configurations

### Cost-Optimized

```bash
# Use cheaper model
OPENAI_MODEL=gpt-3.5-turbo

# Free local embeddings
USE_LOCAL_EMBEDDINGS=true

# Mock robot (no hardware wear)
MOCK_ROBOT=true
```

**Cost**: ~$0.10-0.50/hour

### Performance-Optimized

```bash
# Fastest model
OPENAI_MODEL=gpt-4o

# Best quality embeddings
USE_LOCAL_EMBEDDINGS=false
OPENAI_EMBEDDING_MODEL=text-embedding-3-large

# More threads
THREAD_POOL_SIZE=8

# Skip frames when busy
PROCESS_ALL_INPUTS=false
```

### Development Setup

```bash
# Mock robot
MOCK_ROBOT=true

# Verbose logging
LOG_LEVEL=DEBUG
DIMOS_DEBUG=true

# Enable web UI
ENABLE_WEB_INTERFACE=true
WEB_PORT=8080

# Isolated ROS domain
ROS_DOMAIN_ID=42

# Cheap model
OPENAI_MODEL=gpt-4o-mini
```

### Production Setup

```bash
# Real robot
MOCK_ROBOT=false
GO2_IP=192.168.1.103

# Best model
OPENAI_MODEL=gpt-4o
USE_PLANNING_AGENT=true

# Disable web UI (security)
ENABLE_WEB_INTERFACE=false

# Production logging
LOG_LEVEL=WARNING
DIMOS_DEBUG=false

# Production ROS domain
ROS_DOMAIN_ID=10
```

## Default Values

If a variable is not set, these defaults are used:

| Variable | Default | Source |
|----------|---------|--------|
| `OPENAI_MODEL` | `gpt-4o` | DIMOS |
| `ROS_DOMAIN_ID` | `0` | ROS2 |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | ROS2 |
| `ENABLE_WEB_INTERFACE` | `true` | mission_agent.py |
| `WEB_PORT` | `8080` | mission_agent.py |
| `MOCK_ROBOT` | `false` | mission_agent.py |
| `USE_PLANNING_AGENT` | `false` | mission_agent.py |
| `USE_LOCAL_EMBEDDINGS` | `false` | rag_memory_example.py |
| `RAG_QUERY_N` | `4` | DIMOS |
| `RAG_SIMILARITY_THRESHOLD` | `0.45` | DIMOS |
| `LOG_LEVEL` | `INFO` | Python |
| `AGENT_BACKEND` | `cloud` | mission_agent.py |

## Environment Precedence

Variables are loaded in this order (later overrides earlier):

1. **System defaults** (hardcoded in code)
2. **System environment** (export VAR=value)
3. **`.env` file** (your configuration)
4. **ROS2 launch parameters** (highest priority)

Example:
```bash
# In .env
WEB_PORT=8080

# Override at runtime
ros2 launch shadowhound_mission_agent bringup.launch.py web_port:=9000
# Result: Uses port 9000
```

## Security Best Practices

### ⚠️ NEVER commit `.env` to git!

The `.env` file is in `.gitignore` to prevent accidental commits.

### ✅ DO:
- Use `.env.example` as reference
- Copy `.env.example` to `.env` and customize
- Restrict permissions: `chmod 600 .env`
- Use secrets managers in production (AWS Secrets, Vault, etc.)
- Rotate API keys regularly
- Use different keys for dev/prod

### ❌ DON'T:
- Commit `.env` to version control
- Share API keys in chat/email
- Use production keys in development
- Leave `.env` world-readable

## Validation

Check your configuration:

```bash
# Show loaded environment
source .env
env | grep -E "(OPENAI|GO2|ROS|WEB)" | sort

# Test OpenAI key
python3 -c "
import os
from openai import OpenAI
client = OpenAI()
print('✓ OpenAI key valid')
"

# Test robot connection (if real robot)
ping $GO2_IP

# Test ROS2 domain
ros2 node list  # Should show nodes on your domain
```

## Troubleshooting

### "OPENAI_API_KEY not set"

```bash
# Check if .env exists
ls -la .env

# Check contents
grep OPENAI_API_KEY .env

# Make sure it's not commented
# Wrong: # OPENAI_API_KEY=sk-...
# Right: OPENAI_API_KEY=sk-...

# Test loading
source .env
echo $OPENAI_API_KEY
```

### "Can't connect to robot"

```bash
# Check GO2_IP is set
echo $GO2_IP

# Test network
ping $GO2_IP

# Check robot is powered on
# Check you're on correct network
# Check firewall rules

# Try mock mode first
echo "MOCK_ROBOT=true" >> .env
```

### "Port 8080 already in use"

```bash
# Find what's using the port
sudo lsof -i :8080

# Kill it or change WEB_PORT
echo "WEB_PORT=9000" >> .env
```

### "ROS nodes can't communicate"

```bash
# Check all nodes use same domain
echo $ROS_DOMAIN_ID

# Make sure it's set consistently
# All terminals need: export ROS_DOMAIN_ID=42
# Or add to .env: ROS_DOMAIN_ID=42
```

## Migration from Older Versions

If you have an old configuration, update these:

```bash
# Old variable names → New names
# (None yet, first release)

# Removed variables
# (None yet, first release)

# New required variables
OPENAI_API_KEY=sk-...  # Always required
```

## Advanced: Multiple Environments

Manage multiple configurations:

```bash
# Development
ln -sf .env.development .env
ros2 launch shadowhound_mission_agent bringup.launch.py

# Production
ln -sf .env.production .env
ros2 launch shadowhound_mission_agent bringup.launch.py

# Or specify explicitly
set -a; source .env.development; set +a
ros2 launch shadowhound_mission_agent bringup.launch.py
```

## Getting Help

### Documentation
- **Environment variables**: `.env.example` (this directory)
- **ChatGPT integration**: `docs/CHATGPT_INTEGRATION.md`
- **RAG setup**: `docs/RAG_INTEGRATION.md`
- **Web interface**: `src/shadowhound_mission_agent/WEB_INTERFACE.md`

### Support
- **Issues**: https://github.com/danmartinez78/shadowhound/issues
- **Project context**: `docs/project_context.md`

## Complete Example

```bash
# 1. Copy template
cp .env.example .env

# 2. Edit (minimum config)
cat > .env << 'EOF'
# OpenAI
OPENAI_API_KEY=sk-proj-your-key-here

# Robot (mock for now)
MOCK_ROBOT=true

# ROS2
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Web
ENABLE_WEB_INTERFACE=true
WEB_PORT=8080

# Logging
LOG_LEVEL=INFO
EOF

# 3. Secure it
chmod 600 .env

# 4. Test
source .env
echo "OpenAI key: ${OPENAI_API_KEY:0:10}..."
echo "Mock robot: $MOCK_ROBOT"
echo "Web port: $WEB_PORT"

# 5. Launch
ros2 launch shadowhound_mission_agent bringup.launch.py

# 6. Open browser
xdg-open http://localhost:8080
```

Done! 🚀
