# Start Script System - Quick Reference

## 🚀 One Command to Launch

```bash
./start.sh --dev    # Development mode (mock robot, cheap, easy)
./start.sh --prod   # Production mode (real robot, best quality)
./start.sh          # Interactive mode (asks you questions)
```

## 📁 Files Created

```
shadowhound/
├── start.sh                          # Main launcher (400+ lines)
├── SCRIPTS.md                        # Complete documentation
└── scripts/
    ├── quick-start-dev.sh            # One-line dev start
    ├── quick-start-prod.sh           # One-line prod start
    ├── check-deps.sh                 # Verify dependencies
    └── test-web-only.sh              # Test web server alone
```

## 🎯 What start.sh Does

1. **System Check** ✓
   - ROS2 installed?
   - Python 3 available?
   - colcon build tool?
   - Workspace structure valid?

2. **Configuration Setup** ⚙️
   - .env missing? → Create from template
   - Interactive or auto-select (--dev/--prod)
   - Validate OpenAI API key
   - Load environment variables

3. **Build Workspace** 🔨
   - Check if already built
   - Ask before rebuilding
   - Build shadowhound packages
   - Show build logs

4. **Check Dependencies** 📦
   - Verify Python packages (fastapi, openai, uvicorn)
   - Offer to install missing ones
   - Source workspace correctly

5. **Network Check** 🌐
   - Ping robot IP (if not mock mode)
   - Warn if unreachable
   - Allow override for testing

6. **Pre-flight Summary** 📊
   - Show all configuration
   - Display web dashboard URL
   - List ROS topics
   - Ask for final confirmation

7. **Launch!** 🚀
   - Start ROS2 system
   - Open web interface
   - Handle Ctrl+C gracefully
   - Clean up on exit

## 🎨 Beautiful Output

```
============================================================================
  🐕  ShadowHound - Autonomous Robot Control System
============================================================================

── System Check ────────────────────────────────────────────────────────
✓ ROS2 installed
✓ Python 3.10.12 installed
✓ colcon build tool installed
✓ Workspace structure valid

── Configuration Setup ─────────────────────────────────────────────────
✓ Created .env from development template
✓ Loaded environment variables
✓ OpenAI API key configured
ℹ Using MOCK robot mode (no hardware needed)

── Pre-flight Summary ──────────────────────────────────────────────────

Configuration:
  • Mode: development
  • Mock Robot: true
  • Web Interface: true
  • Web Port: 8080
  • ROS Domain: 42
  • OpenAI Model: gpt-4o-mini

🌐 Web Dashboard: http://localhost:8080

ROS Topics:
  • Commands: /mission_command
  • Status: /mission_status

Ready to launch!
Press Enter to start (or Ctrl+C to cancel)...
```

## 💡 Common Usage Patterns

### First Time User
```bash
./start.sh
# → Interactive mode guides you through setup
# → Creates .env from template
# → Asks to edit API key
# → Builds workspace
# → Launches system
```

### Daily Development
```bash
./scripts/quick-start-dev.sh
# → Instant launch with dev settings
```

### Testing Web UI
```bash
./scripts/test-web-only.sh
# → Web server only, no ROS overhead
# → Good for debugging web issues
```

### Checking Setup
```bash
./scripts/check-deps.sh
# → Shows what's installed
# → Identifies missing packages
# → Validates configuration
```

### Production Deployment
```bash
# 1. Configure
cp .env.production .env
nano .env  # Add API key and robot IP

# 2. Launch
./start.sh --prod
```

## 🛠️ Options Reference

| Option | Description | Example |
|--------|-------------|---------|
| `--dev` | Development mode | `./start.sh --dev` |
| `--prod` | Production mode | `./start.sh --prod` |
| `--mock` | Force mock robot | `./start.sh --mock` |
| `--no-web` | Disable web UI | `./start.sh --no-web` |
| `--web-port N` | Custom port | `./start.sh --web-port 9000` |
| `--help` | Show help | `./start.sh --help` |

## 🔧 Troubleshooting Quick Fixes

**"ROS2 not found"**
```bash
source /opt/ros/humble/setup.bash
```

**"OpenAI API key not set"**
```bash
nano .env
# Add: OPENAI_API_KEY=sk-your-key
```

**"Cannot reach robot"**
```bash
./start.sh --mock  # Use mock mode
```

**"Port 8080 in use"**
```bash
./start.sh --web-port 9000  # Use different port
```

**"Missing Python packages"**
```bash
# Script offers to install automatically
# Or manually: pip3 install fastapi uvicorn openai
```

## 📚 Full Documentation

- **SCRIPTS.md** - Complete guide with examples
- **ENV_CONFIG.md** - Environment configuration reference
- **README.md** - Main project documentation
- **WEB_INTERFACE.md** - Web dashboard guide

## ✨ Key Features

- ✅ **Smart** - Detects what's missing and helps fix it
- ✅ **Safe** - Validates before launching
- ✅ **Fast** - Quick starts for daily use
- ✅ **Clear** - Beautiful colored output
- ✅ **Helpful** - Suggests fixes for common issues
- ✅ **Flexible** - Multiple modes and options
- ✅ **Complete** - Handles entire workflow

## 🎯 Design Philosophy

1. **Make the simple easy** - One command for common cases
2. **Make the complex possible** - Options for custom needs
3. **Fail helpfully** - Clear errors with suggestions
4. **Guide don't block** - Ask questions, offer to help
5. **Beauty matters** - Nice output makes people happy 🎨
