# ShadowHound Scripts

Helper scripts for easy setup and operation of the ShadowHound system.

## Main Scripts

### 🚀 `start.sh` - Main Launch Script

The primary script for starting ShadowHound. It handles:
- System dependency checks
- Configuration setup (.env creation)
- Workspace building
- Network validation
- System launch

**Usage:**
```bash
./start.sh [OPTIONS]
```

**Options:**
- `--dev` - Use development configuration (mock robot, cheap model)
- `--prod` - Use production configuration (real robot, best model)
- `--mock` - Force mock robot mode
- `--no-web` - Disable web interface
- `--web-port N` - Set web port (default: 8080)
- `--help` - Show help message

**Examples:**
```bash
# Interactive mode (asks questions)
./start.sh

# Quick development start
./start.sh --dev

# Production with real robot
./start.sh --prod

# Development without web interface
./start.sh --dev --no-web

# Custom port
./start.sh --dev --web-port 9000
```

**What it does:**
1. ✓ Checks ROS2, Python, colcon are installed
2. ✓ Validates workspace structure
3. ✓ Creates .env from template if missing
4. ✓ Validates OpenAI API key
5. ✓ Builds ShadowHound packages
6. ✓ Installs missing Python dependencies
7. ✓ Checks network connection to robot (if not mock)
8. ✓ Shows configuration summary
9. ✓ Launches the system!

## Quick Start Scripts

### 🔧 `scripts/quick-start-dev.sh`

One-command development start with optimal settings:
- Mock robot (no hardware needed)
- Cheap model (gpt-4o-mini)
- Free local embeddings
- Web interface enabled
- Verbose logging

```bash
./scripts/quick-start-dev.sh
```

### 🏭 `scripts/quick-start-prod.sh`

One-command production start with security:
- Real robot connection
- Best model (gpt-4o)
- Production logging
- Security-focused defaults

```bash
./scripts/quick-start-prod.sh
```

## Testing & Debugging Scripts

### 🔍 `scripts/check-deps.sh`

Check all dependencies without building or launching:
```bash
./scripts/check-deps.sh
```

Shows status of:
- ROS2 installation
- Python version
- colcon build tool
- Python packages (rclpy, fastapi, openai, etc.)
- Git submodules
- Configuration files
- Workspace build status

### 🌐 `scripts/test-web-only.sh`

Test the web interface in isolation (without ROS/robot):
```bash
./scripts/test-web-only.sh
```

Useful for:
- Checking if web server works
- Debugging web UI issues
- Testing without full system
- Verifying port is available

Opens web dashboard at http://localhost:8080

## Workflow Examples

### First Time Setup

```bash
# 1. Check if everything is ready
./scripts/check-deps.sh

# 2. Start in development mode (creates .env, builds, launches)
./start.sh --dev

# 3. Open browser to http://localhost:8080
# 4. Test with: "patrol around the room"
```

### Daily Development

```bash
# Quick start (already configured)
./scripts/quick-start-dev.sh
```

### Testing Web Interface

```bash
# Test web server without ROS overhead
./scripts/test-web-only.sh

# If that works, try full system
./start.sh --dev
```

### Production Deployment

```bash
# 1. Check dependencies on target system
./scripts/check-deps.sh

# 2. Copy production template and configure
cp .env.production .env
nano .env  # Set OPENAI_API_KEY and GO2_IP

# 3. Launch with production settings
./start.sh --prod
```

### Troubleshooting

```bash
# Check what's missing
./scripts/check-deps.sh

# Test web interface alone
./scripts/test-web-only.sh

# Try with verbose output
./start.sh --dev 2>&1 | tee launch.log

# Check ROS topics
ros2 topic list
ros2 topic echo /mission_status
```

## Configuration Files Used

The scripts work with these configuration files:

- **`.env`** - Your active configuration (created by start.sh)
- **`.env.example`** - Master reference with all variables
- **`.env.development`** - Development template
- **`.env.production`** - Production template

See `ENV_CONFIG.md` for complete configuration guide.

## Script Features

### Smart Configuration
- Detects if .env exists
- Offers template selection (dev/prod/custom)
- Validates required variables
- Helps edit configuration

### Dependency Management
- Checks system requirements
- Validates Python packages
- Offers to install missing packages
- Verifies workspace structure

### Network Validation
- Pings robot IP before launch
- Skips check for mock mode
- Provides helpful error messages
- Allows override if needed

### User-Friendly Output
- Color-coded status messages
- Emojis for quick scanning
- Progress indicators
- Clear error messages
- Helpful suggestions

### Safety Features
- Confirmation before launch
- Graceful shutdown handling
- Process cleanup on exit
- Validation before operations

## Common Issues

### "ROS2 not found"
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Or add to .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### "OpenAI API key not set"
```bash
# Edit .env file
nano .env

# Add your key
OPENAI_API_KEY=sk-your-actual-key-here
```

### "Cannot reach robot"
```bash
# Use mock mode for testing
./start.sh --mock

# Or check network
ping 192.168.1.103
```

### "Build failed"
```bash
# Check detailed logs
cat /tmp/colcon_build.log

# Try clean build
rm -rf build install log
./start.sh
```

### "Port 8080 already in use"
```bash
# Use different port
./start.sh --web-port 9000

# Or kill existing process
lsof -ti:8080 | xargs kill -9
```

## Tips

1. **First time?** Use `./start.sh` (interactive mode)
2. **Daily use?** Use `./scripts/quick-start-dev.sh`
3. **Web issues?** Try `./scripts/test-web-only.sh` first
4. **Checking setup?** Run `./scripts/check-deps.sh`
5. **Production?** Use `./start.sh --prod` after configuring .env

## See Also

- `README.md` - Main project documentation
- `ENV_CONFIG.md` - Complete environment variable guide
- `WEB_INTERFACE.md` - Web dashboard documentation
- `docs/project_context.md` - Architecture and design
