# DIMOS Web Interface Integration

**Status**: ❌ **Not Currently Integrated**  
**Available**: ✅ Yes, DIMOS provides web interfaces  
**Priority**: 🔶 Medium - Nice to have for monitoring/control

---

## Current Implementation

### What We Have Now

Our current implementation uses **ROS2 topics only**:

```
User → ros2 topic pub /mission_command → Mission Agent → Robot
         ↓
    ros2 topic echo /mission_status ← Status Updates
```

**Interfaces**:
- Command line (ROS2 CLI)
- Python scripts (rclpy)
- Any ROS2 node

**Limitations**:
- ❌ No web dashboard
- ❌ No video streaming UI
- ❌ No visual telemetry
- ❌ No remote browser access

---

## What DIMOS Web Interface Provides

### Available Components

1. **FastAPIServer** (`dimos/web/fastapi_server.py`)
   - Modern async Python web framework
   - REST API endpoints
   - WebSocket support

2. **FlaskServer** (`dimos/web/flask_server.py`)
   - Traditional Python web framework
   - Simpler setup
   - Good for basic UIs

3. **RobotWebInterface** (`dimos/web/robot_web_interface.py`)
   - Wrapper around FastAPIServer
   - Robot-specific endpoints
   - Stream management (video, telemetry)

4. **WebSocket Visualization** (`dimos/web/websocket_vis/`)
   - Real-time data streaming
   - Live updates without polling
   - Efficient for high-frequency data

### Features Available

✅ **Video Streaming**
- Camera feed display
- Multiple camera support
- WebRTC integration

✅ **Telemetry Dashboard**
- Robot state visualization
- Sensor data plots
- Real-time updates

✅ **Command Interface**
- Web-based command sending
- Mission templates
- Skill execution buttons

✅ **Status Monitoring**
- Mission progress
- Error reporting
- System health

---

## Integration Options

### Option 1: Add Web Interface to Mission Agent (Recommended)

**Approach**: Extend mission agent to also run web server

**Benefits**:
- Single node deployment
- Unified configuration
- Easy to launch together

**Architecture**:
```
┌────────────────────────────────────────────┐
│      Mission Agent Node                    │
│  ┌──────────────────────────────────────┐  │
│  │  ROS2 Interface                      │  │
│  │  - /mission_command (topic)          │  │
│  │  - /mission_status (topic)           │  │
│  └──────────────────────────────────────┘  │
│  ┌──────────────────────────────────────┐  │
│  │  Web Interface (NEW)                 │  │
│  │  - FastAPI server on port 5555       │  │
│  │  - REST API: POST /mission           │  │
│  │  - WebSocket: /ws/status             │  │
│  │  - Static UI: /                      │  │
│  └──────────────────────────────────────┘  │
│               ↓                             │
│         DIMOS Agent                         │
└────────────────────────────────────────────┘
```

**Implementation**:

```python
# In mission_agent.py
from dimos.web.robot_web_interface import RobotWebInterface
import threading

class MissionAgentNode(Node):
    def __init__(self):
        super().__init__("shadowhound_mission_agent")
        
        # ... existing initialization ...
        
        # Add web interface
        if self.get_parameter("enable_web_interface").value:
            self._init_web_interface()
    
    def _init_web_interface(self):
        """Initialize DIMOS web interface."""
        port = self.get_parameter("web_port").value
        
        # Create web interface
        self.web_interface = RobotWebInterface(
            port=port,
            text_streams={"mission_status": self._get_status_stream}
        )
        
        # Add custom endpoints
        @self.web_interface.app.post("/mission")
        async def send_mission(command: str):
            """Send mission command via web API."""
            msg = String()
            msg.data = command
            # Publish to ROS topic
            self.mission_callback(msg)
            return {"status": "accepted", "command": command}
        
        # Start web server in background thread
        self.web_thread = threading.Thread(
            target=self.web_interface.run,
            daemon=True
        )
        self.web_thread.start()
        
        self.get_logger().info(
            f"Web interface running on http://0.0.0.0:{port}"
        )
```

### Option 2: Separate Web Interface Node

**Approach**: Create dedicated `shadowhound_web_interface` package

**Benefits**:
- Clean separation of concerns
- Independent scaling
- Optional component

**Architecture**:
```
┌─────────────────┐         ┌──────────────────┐
│  Web Interface  │ ◄─────► │  Mission Agent   │
│  Node           │  ROS2   │  Node            │
│  (port 5555)    │ Topics  │  (ROS2 + DIMOS)  │
└─────────────────┘         └──────────────────┘
        ↑
    Browser Users
```

**Implementation**:

```python
# New package: shadowhound_web_interface
class WebInterfaceNode(Node):
    def __init__(self):
        super().__init__("shadowhound_web_interface")
        
        # Subscribe to status
        self.status_sub = self.create_subscription(
            String, "/mission_status",
            self.status_callback, 10
        )
        
        # Publish commands
        self.cmd_pub = self.create_publisher(
            String, "/mission_command", 10
        )
        
        # Start web server
        self.web = RobotWebInterface(port=5555)
        
        @self.web.app.post("/mission")
        async def send_mission(command: str):
            msg = String()
            msg.data = command
            self.cmd_pub.publish(msg)
            return {"status": "sent"}
        
        @self.web.app.get("/status")
        async def get_status():
            return {"status": self.latest_status}
```

### Option 3: Use DIMOS Web Interface As-Is

**Approach**: Launch DIMOS's web interface alongside our nodes

**Benefits**:
- Zero custom code
- Use DIMOS features directly
- Proven implementation

**Limitations**:
- Less ShadowHound-specific customization
- May need to learn DIMOS API

---

## Recommended Integration Path

### Phase 1: Basic Web UI (v0.2.0)

**Add to mission_agent.py**:

```python
# Add parameter
self.declare_parameter("enable_web_interface", False)
self.declare_parameter("web_port", 5555)

# Add web interface initialization
if self.get_parameter("enable_web_interface").value:
    self._init_web_interface()
```

**Features**:
- Simple REST API for sending commands
- WebSocket for status streaming
- Basic HTML dashboard

**Launch**:
```bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    enable_web_interface:=true \
    web_port:=5555
```

**Access**:
```
http://localhost:5555/           # Dashboard
http://localhost:5555/mission    # POST commands
ws://localhost:5555/ws/status    # WebSocket status
```

### Phase 2: Video Streaming (v0.3.0)

**Add video feed**:
```python
self.web_interface = RobotWebInterface(
    port=port,
    text_streams={"mission_status": self._get_status_stream},
    video_streams={"camera": self.robot.get_camera_feed}
)
```

**Features**:
- Live robot camera view
- Multiple camera angles
- Recording capability

### Phase 3: Advanced Dashboard (v0.4.0)

**Features**:
- Telemetry graphs (speed, battery, etc.)
- Skill execution buttons
- Mission templates
- Multi-robot view

---

## Example Integration Code

### Minimal Web Interface Addition

```python
# Add to mission_agent.py

from dimos.web.robot_web_interface import RobotWebInterface
import threading
from typing import Optional

class MissionAgentNode(Node):
    def __init__(self):
        super().__init__("shadowhound_mission_agent")
        
        # Declare web interface parameters
        self.declare_parameter("enable_web_interface", False)
        self.declare_parameter("web_port", 5555)
        
        # ... existing initialization ...
        
        # Initialize web interface if enabled
        self.web_interface: Optional[RobotWebInterface] = None
        if self.get_parameter("enable_web_interface").value:
            self._init_web_interface()
    
    def _init_web_interface(self):
        """Initialize DIMOS web interface for browser access."""
        try:
            port = self.get_parameter("web_port").value
            
            # Create web interface
            self.web_interface = RobotWebInterface(
                port=port,
                text_streams={
                    "mission_status": lambda: self.latest_status
                }
            )
            
            # Add mission command endpoint
            @self.web_interface.app.post("/api/mission")
            async def send_mission_command(command: dict):
                """Send mission command via REST API."""
                cmd = command.get("command", "")
                
                # Create and send ROS message
                msg = String()
                msg.data = cmd
                
                # Trigger mission callback
                self.mission_callback(msg)
                
                return {
                    "success": True,
                    "command": cmd,
                    "message": "Mission command sent"
                }
            
            # Start web server in background
            self.web_thread = threading.Thread(
                target=self.web_interface.run,
                daemon=True
            )
            self.web_thread.start()
            
            self.get_logger().info(
                f"🌐 Web interface available at http://0.0.0.0:{port}"
            )
            self.get_logger().info(
                f"   Dashboard: http://0.0.0.0:{port}/"
            )
            self.get_logger().info(
                f"   API: http://0.0.0.0:{port}/api/mission"
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to start web interface: {e}")
    
    def mission_callback(self, msg: String):
        """Handle incoming mission commands (ROS2 or Web)."""
        # Store latest status for web interface
        self.latest_status = f"EXECUTING: {msg.data}"
        
        # ... existing implementation ...
```

### Simple HTML Dashboard

```html
<!-- templates/index.html -->
<!DOCTYPE html>
<html>
<head>
    <title>ShadowHound Mission Control</title>
    <style>
        body { font-family: Arial; max-width: 800px; margin: 50px auto; }
        .status { padding: 20px; background: #f0f0f0; border-radius: 5px; }
        .command-box { margin: 20px 0; }
        button { padding: 10px 20px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; }
        button:hover { background: #0056b3; }
    </style>
</head>
<body>
    <h1>🐕 ShadowHound Mission Control</h1>
    
    <div class="status">
        <h3>Status</h3>
        <p id="status">Waiting for updates...</p>
    </div>
    
    <div class="command-box">
        <h3>Send Mission Command</h3>
        <input type="text" id="command" placeholder="Enter command (e.g., 'stand up')" style="width: 500px; padding: 10px;">
        <button onclick="sendCommand()">Send</button>
    </div>
    
    <div>
        <h3>Quick Commands</h3>
        <button onclick="sendQuick('stand up')">Stand Up</button>
        <button onclick="sendQuick('sit down')">Sit Down</button>
        <button onclick="sendQuick('wave hello')">Wave Hello</button>
        <button onclick="sendQuick('dance 1')">Dance</button>
    </div>
    
    <script>
        // WebSocket connection for real-time status
        const ws = new WebSocket('ws://localhost:5555/ws/status');
        
        ws.onmessage = (event) => {
            document.getElementById('status').textContent = event.data;
        };
        
        function sendCommand() {
            const cmd = document.getElementById('command').value;
            sendQuick(cmd);
        }
        
        function sendQuick(cmd) {
            fetch('/api/mission', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({command: cmd})
            })
            .then(r => r.json())
            .then(data => {
                console.log('Command sent:', data);
                document.getElementById('command').value = '';
            });
        }
    </script>
</body>
</html>
```

---

## Summary

### Current State
- ❌ No web interface integrated yet
- ✅ ROS2 topics only
- ✅ Works well for command-line and scripts

### Why Add Web Interface?
- 🌐 Browser-based access (any device)
- 👁️ Visual feedback and monitoring
- 📹 Video streaming capability
- 🎮 User-friendly control panel
- 📊 Telemetry visualization

### Next Steps
1. **Decide**: Option 1 (extend mission_agent) or Option 2 (separate node)?
2. **Implement**: Add web interface initialization
3. **Test**: Verify REST API and WebSocket
4. **Document**: Update AGENT_DESIGN.md with web features
5. **Deploy**: Test on laptop with real robot

### Recommendation
**Start with Option 1** (extend mission_agent) because:
- Simpler deployment (one node)
- Easier configuration
- Can always split later if needed
- Aligns with "minimal integration" philosophy

Would you like me to implement the web interface integration?
