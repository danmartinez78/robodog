# WebRTC Direct Test - Quick Start

## One-Line Setup

```bash
./scripts/setup_webrtc_test.sh
```

This interactive script will:
1. ✅ Ask for your robot's WiFi IP
2. ✅ Test connectivity
3. ✅ Create `.env.webrtc_test` automatically
4. ✅ No manual file editing needed!

---

## Running the Test

### Terminal 1: Launch SDK Driver
```bash
./scripts/test_webrtc_direct.sh
```

### Terminal 2: Test Commands
```bash
source .shadowhound_env
./scripts/test_commands.sh sit     # Robot sits! 🐕⬇️
./scripts/test_commands.sh stand   # Robot stands! 🐕
./scripts/test_commands.sh wave    # Robot waves! 🐕👋
```

---

## What Gets Configured

The setup script creates `.env.webrtc_test` with:
- ✅ `GO2_IP` - Your robot's WiFi IP
- ✅ `CONN_TYPE=webrtc` - WebRTC mode
- ✅ `ROS_DOMAIN_ID=0` - ROS network isolation
- ✅ `RMW_IMPLEMENTATION` - CycloneDDS middleware

**No manual editing needed!** The test script automatically sources this file.

---

## Files

- **`scripts/setup_webrtc_test.sh`** - Interactive setup wizard
- **`.env.webrtc_test`** - Test-specific configuration (auto-created)
- **`scripts/test_webrtc_direct.sh`** - Main test launcher
- **`scripts/test_commands.sh`** - Quick command tester

---

## Workflow

```
1. Setup (one time):
   ./scripts/setup_webrtc_test.sh
   
2. Test (anytime):
   ./scripts/test_webrtc_direct.sh
   
3. Send commands:
   ./scripts/test_commands.sh sit
```

---

## Troubleshooting

### "Robot not reachable"
```bash
# Re-run setup to change IP
./scripts/setup_webrtc_test.sh
```

### "Cannot find .env.webrtc_test"
```bash
# Setup will auto-create it
./scripts/setup_webrtc_test.sh
```

### "Wrong IP configured"
```bash
# Just run setup again
./scripts/setup_webrtc_test.sh
```

---

## Complete Documentation

See **`docs/WEBRTC_DIRECT_TEST.md`** for full testing guide.
