#!/usr/bin/env python3
"""Web interface for ShadowHound mission control.

Provides a FastAPI-based web server that can be embedded in any ROS2 node.
Designed to be reusable and independent of specific node implementation.
"""

from typing import Callable, Optional, Dict, Any
import asyncio
import threading
import logging
import os
import base64
from pathlib import Path
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, File, UploadFile, Form
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from pydantic import BaseModel


class MissionCommand(BaseModel):
    """Mission command request model."""

    command: str
    priority: Optional[int] = 0


class MissionResponse(BaseModel):
    """Mission command response model."""

    success: bool
    message: str
    command: str


class WebInterface:
    """Reusable web interface for robot mission control.

    This class provides a FastAPI server that can be embedded in any ROS2 node.
    It handles HTTP requests and WebSocket connections for real-time updates.

    Example:
        >>> # In your ROS node
        >>> def handle_command(cmd: str) -> dict:
        ...     # Your command processing logic
        ...     return {"success": True, "message": "Command executed"}
        >>>
        >>> web = WebInterface(
        ...     command_callback=handle_command,
        ...     port=8080
        ... )
        >>> web.start()
    """

    def __init__(
        self,
        command_callback: Callable[[str], Dict[str, Any]],
        port: int = 8080,
        host: str = "0.0.0.0",
        logger: Optional[logging.Logger] = None,
    ):
        """Initialize web interface.

        Args:
            command_callback: Function to call when mission command received.
                             Should accept (command: str) and return dict with
                             'success' and 'message' keys.
            port: Port to run web server on (default: 8080)
            host: Host to bind to (default: 0.0.0.0 for all interfaces)
            logger: Optional logger instance
        """
        self.command_callback = command_callback
        self.port = port
        self.host = host
        self.logger = logger or logging.getLogger(__name__)

        # WebSocket connections for real-time status updates
        self.active_connections: list[WebSocket] = []

        # Mock image storage (for testing without robot)
        self.mock_images: Dict[str, bytes] = {}  # camera -> image_data
        self.mock_image_dir = Path.home() / ".shadowhound" / "mock_images"
        self.mock_image_dir.mkdir(parents=True, exist_ok=True)

        # Server state
        self.server = None
        self.server_thread = None
        self.app = self._create_app()

    def _create_app(self) -> FastAPI:
        """Create FastAPI application with routes."""
        app = FastAPI(
            title="ShadowHound Mission Control",
            description="Web interface for autonomous robot control",
            version="0.1.0",
        )

        # Enable CORS for browser access
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # Register routes
        @app.get("/", response_class=HTMLResponse)
        async def root():
            """Serve main dashboard."""
            return self._get_dashboard_html()

        @app.get("/api/health")
        async def health():
            """Health check endpoint."""
            return {"status": "healthy", "service": "shadowhound-web"}

        @app.post("/api/mission", response_model=MissionResponse)
        async def send_mission(cmd: MissionCommand):
            """Send mission command to robot."""
            try:
                result = self.command_callback(cmd.command)

                # Broadcast to WebSocket clients
                await self.broadcast(f"COMMAND: {cmd.command}")

                return MissionResponse(
                    success=result.get("success", False),
                    message=result.get("message", "Unknown result"),
                    command=cmd.command,
                )
            except Exception as e:
                self.logger.error(f"Error executing command: {e}")
                return MissionResponse(
                    success=False, message=str(e), command=cmd.command
                )

        @app.post("/api/mock/image")
        async def upload_mock_image(
            image: UploadFile = File(...), camera: str = Form("front")
        ):
            """Upload mock image for testing vision skills without robot."""
            try:
                # Read image data
                image_data = await image.read()

                # Save to disk
                filename = f"mock_{camera}_{image.filename}"
                filepath = self.mock_image_dir / filename
                with open(filepath, "wb") as f:
                    f.write(image_data)

                # Store in memory for quick access
                self.mock_images[camera] = image_data

                self.logger.info(f"Mock image uploaded: {filename} ({len(image_data)} bytes)")

                # Broadcast to WebSocket clients
                await self.broadcast(f"MOCK_IMAGE_UPLOADED: {camera} - {filename}")

                return JSONResponse(
                    content={
                        "success": True,
                        "message": f"Mock image uploaded for {camera} camera",
                        "filename": filename,
                        "size": len(image_data),
                        "camera": camera,
                    }
                )
            except Exception as e:
                self.logger.error(f"Error uploading mock image: {e}")
                return JSONResponse(
                    status_code=500,
                    content={
                        "success": False,
                        "message": str(e),
                    },
                )

        @app.get("/api/mock/image/{camera}")
        async def get_mock_image(camera: str):
            """Get mock image for specified camera."""
            if camera not in self.mock_images:
                return JSONResponse(
                    status_code=404,
                    content={
                        "success": False,
                        "message": f"No mock image uploaded for {camera} camera",
                    },
                )

            image_data = self.mock_images[camera]
            # Return base64 encoded image
            import base64

            encoded = base64.b64encode(image_data).decode("utf-8")
            return JSONResponse(
                content={
                    "success": True,
                    "camera": camera,
                    "image": encoded,
                    "size": len(image_data),
                }
            )

        @app.websocket("/ws/status")
        async def websocket_status(websocket: WebSocket):
            """WebSocket endpoint for real-time status updates."""
            await websocket.accept()
            self.active_connections.append(websocket)

            try:
                # Keep connection alive
                while True:
                    # Wait for messages (optional ping/pong)
                    data = await websocket.receive_text()
                    # Echo back for testing
                    await websocket.send_text(f"Echo: {data}")
            except WebSocketDisconnect:
                self.active_connections.remove(websocket)
                self.logger.info("WebSocket client disconnected")

        return app

    def _get_dashboard_html(self) -> str:
        """Generate dashboard HTML."""
        return """
<!DOCTYPE html>
<html>
<head>
    <title>shadowhound mission control</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Courier New', 'Monaco', monospace;
            background: #1a1a1a;
            color: #c0c0c0;
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1600px;
            margin: 0 auto;
        }
        
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 15px 20px;
            background: #0d0d0d;
            border: 1px solid #333;
            margin-bottom: 20px;
        }
        
        .header-left h1 {
            font-size: 1.5em;
            color: #5cb85c;
            letter-spacing: 2px;
            margin-bottom: 5px;
        }
        
        .header-left .subtitle {
            font-size: 0.85em;
            color: #666;
        }
        
        .header-right {
            text-align: right;
        }
        
        .connection-status {
            display: inline-block;
            padding: 6px 12px;
            border: 1px solid;
            font-size: 0.85em;
            font-weight: bold;
        }
        
        .connected {
            background: #0d0d0d;
            color: #5cb85c;
            border-color: #5cb85c;
        }
        
        .disconnected {
            background: #0d0d0d;
            color: #ff0000;
            border-color: #ff0000;
        }
        
        .main-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            grid-template-rows: auto auto;
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .panel {
            background: #0d0d0d;
            border: 1px solid #333;
            padding: 20px;
        }
        
        .panel-header {
            font-size: 0.9em;
            color: #5cb85c;
            margin-bottom: 15px;
            padding-bottom: 10px;
            border-bottom: 1px solid #333;
        }
        
        .camera-panel {
            grid-column: 1;
            grid-row: 1;
            min-height: 400px;
        }
        
        .camera-feed {
            width: 100%;
            aspect-ratio: 16/9;
            background: #000;
            border: 1px solid #333;
            display: flex;
            align-items: center;
            justify-content: center;
            position: relative;
            overflow: hidden;
        }
        
        .camera-feed img {
            width: 100%;
            height: 100%;
            object-fit: contain;
        }
        
        .camera-placeholder {
            color: #333;
            font-size: 1.2em;
            text-align: center;
        }
        
        .upload-panel {
            margin-top: 15px;
            padding: 15px;
            background: #1a1a1a;
            border: 1px solid #333;
        }
        
        .upload-label {
            font-size: 0.85em;
            color: #888;
            margin-bottom: 10px;
            display: block;
        }
        
        .file-input-wrapper {
            position: relative;
            display: inline-block;
            width: 100%;
        }
        
        .file-input-wrapper input[type="file"] {
            position: absolute;
            opacity: 0;
            width: 100%;
            height: 100%;
            cursor: pointer;
        }
        
        .file-input-button {
            display: block;
            width: 100%;
            padding: 12px;
            background: #0d0d0d;
            border: 1px dashed #555;
            color: #c0c0c0;
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            text-align: center;
            cursor: pointer;
            transition: all 0.2s;
        }
        
        .file-input-button:hover {
            background: #1a1a1a;
            border-color: #888;
        }
        
        .mission-panel {
            grid-column: 2;
            grid-row: 1 / span 2;
            display: flex;
            flex-direction: column;
        }
        
        .status-display {
            flex: 1;
            background: #000;
            border: 1px solid #333;
            padding: 15px;
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            color: #c0c0c0;
            overflow-y: auto;
            max-height: 300px;
            margin-bottom: 20px;
        }
        
        .status-display .log-entry {
            padding: 3px 0;
            line-height: 1.5;
        }
        
        .command-input {
            width: 100%;
            padding: 12px;
            background: #000;
            border: 1px solid #333;
            color: #5cb85c;
            font-family: 'Courier New', monospace;
            font-size: 1em;
        }
        
        .command-input:focus {
            outline: none;
            border-color: #555;
            background: #0d0d0d;
        }
        
        .command-input::placeholder {
            color: #333;
        }
        
        .btn {
            padding: 12px 24px;
            border: 1px solid #333;
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            cursor: pointer;
            transition: all 0.2s;
            margin-top: 15px;
            background: #0d0d0d;
        }
        
        .btn-primary {
            border-color: #5cb85c;
            color: #5cb85c;
        }
        
        .btn-primary:hover {
            background: #1a1a1a;
        }
        
        .btn-primary:active {
            background: #000;
        }
        
        .diagnostics-panel {
            grid-column: 1;
            grid-row: 2;
        }
        
        .diag-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }
        
        .diag-item {
            padding: 12px;
            background: #1a1a1a;
            border: 1px solid #333;
        }
        
        .diag-label {
            font-size: 0.75em;
            color: #666;
            margin-bottom: 5px;
        }
        
        .diag-value {
            font-size: 1.1em;
            color: #5cb85c;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="container">
        <!-- Header -->
        <div class="header">
            <div class="header-left">
                <h1>shadowhound@mission-control:~$</h1>
                <div class="subtitle">autonomous robot control interface</div>
            </div>
            <div class="header-right">
                <span id="wsStatus" class="connection-status disconnected">[OFFLINE]</span>
            </div>
        </div>
        
        <!-- Main Grid -->
        <div class="main-grid">
            <!-- Camera Feed Panel -->
            <div class="panel camera-panel">
                <div class="panel-header">camera_feed:</div>
                <div class="camera-feed" id="cameraFeed">
                    <div class="camera-placeholder">[ no signal ]</div>
                </div>
                
                <!-- Mock Image Upload -->
                <div class="upload-panel">
                    <label class="upload-label">mock_image (testing):</label>
                    <div class="file-input-wrapper">
                        <input type="file" id="imageUpload" accept="image/*" onchange="uploadMockImage(event)">
                        <div class="file-input-button">[ upload test image ]</div>
                    </div>
                </div>
            </div>
            
            <!-- Mission Control Panel -->
            <div class="panel mission-panel">
                <div class="panel-header">mission_log:</div>
                <div class="status-display" id="status">
                    <div class="log-entry">system ready. awaiting commands.</div>
                </div>
                <input 
                    type="text" 
                    id="commandInput" 
                    class="command-input" 
                    placeholder="$ enter command..."
                    onkeypress="if(event.key === 'Enter') sendCommand()"
                >
                <button class="btn btn-primary" onclick="sendCommand()">[ execute ]</button>
            </div>
            
            <!-- Diagnostics Panel -->
            <div class="panel diagnostics-panel">
                <div class="panel-header">diagnostics:</div>
                <div class="diag-grid">
                    <div class="diag-item">
                        <div class="diag-label">status:</div>
                        <div class="diag-value" id="diagStatus">IDLE</div>
                    </div>
                    <div class="diag-item">
                        <div class="diag-label">battery:</div>
                        <div class="diag-value" id="diagBattery">--</div>
                    </div>
                    <div class="diag-item">
                        <div class="diag-label">position:</div>
                        <div class="diag-value" id="diagPosition">0.0, 0.0</div>
                    </div>
                    <div class="diag-item">
                        <div class="diag-label">connection:</div>
                        <div class="diag-value" id="diagConnection">STANDBY</div>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        let ws;
        const statusDiv = document.getElementById('status');
        const wsStatusSpan = document.getElementById('wsStatus');
        const commandInput = document.getElementById('commandInput');
        const cameraFeed = document.getElementById('cameraFeed');
        const diagStatus = document.getElementById('diagStatus');
        const diagConnection = document.getElementById('diagConnection');
        
        // WebSocket connection
        function connectWebSocket() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            ws = new WebSocket(`${protocol}//${window.location.host}/ws/status`);
            
            ws.onopen = () => {
                console.log('WebSocket connected');
                wsStatusSpan.textContent = '[ONLINE]';
                wsStatusSpan.className = 'connection-status connected';
                diagConnection.textContent = 'ONLINE';
                addLogEntry('websocket connected');
            };
            
            ws.onmessage = (event) => {
                console.log('Status update:', event.data);
                
                // Check if this is a camera frame
                if (event.data.startsWith('CAMERA_FRAME:')) {
                    const base64Data = event.data.substring(13); // Remove 'CAMERA_FRAME:' prefix
                    const imgSrc = 'data:image/jpeg;base64,' + base64Data;
                    cameraFeed.innerHTML = `<img src="${imgSrc}" alt="Robot camera feed">`;
                    return; // Don't log camera frames
                }
                
                // Regular status message
                addLogEntry(event.data.toLowerCase());
                
                // Update diagnostics
                if (event.data.includes('EXECUTING')) {
                    diagStatus.textContent = 'ACTIVE';
                } else if (event.data.includes('COMPLETED')) {
                    diagStatus.textContent = 'IDLE';
                }
            };
            
            ws.onclose = () => {
                console.log('WebSocket disconnected');
                wsStatusSpan.textContent = '[OFFLINE]';
                wsStatusSpan.className = 'connection-status disconnected';
                diagConnection.textContent = 'OFFLINE';
                addLogEntry('websocket disconnected', 'error');
                // Reconnect after 3 seconds
                setTimeout(connectWebSocket, 3000);
            };
            
            ws.onerror = (error) => {
                console.error('WebSocket error:', error);
                addLogEntry('websocket error', 'error');
            };
        }
        
        function addLogEntry(message, type = 'info') {
            const entry = document.createElement('div');
            entry.className = 'log-entry';
            
            const timestamp = new Date().toLocaleTimeString('en-US', { hour12: false });
            const prefix = type === 'error' ? '[!]' : '[>]';
            
            entry.textContent = `${timestamp} ${prefix} ${message}`;
            
            if (type === 'error') {
                entry.style.color = '#ff0000';
            }
            
            statusDiv.appendChild(entry);
            statusDiv.scrollTop = statusDiv.scrollHeight;
            
            // Keep only last 50 entries
            while (statusDiv.children.length > 50) {
                statusDiv.removeChild(statusDiv.firstChild);
            }
        }
        
        // Send command via REST API
        async function sendCommand() {
            const command = commandInput.value.trim();
            if (!command) {
                addLogEntry('no command entered', 'error');
                return;
            }
            
            addLogEntry(`executing: ${command}`);
            diagStatus.textContent = 'ACTIVE';
            commandInput.value = '';
            
            try {
                const response = await fetch('/api/mission', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({command: command})
                });
                
                const data = await response.json();
                console.log('Response:', data);
                
                if (data.success) {
                    addLogEntry(`completed: ${data.message}`);
                    diagStatus.textContent = 'IDLE';
                } else {
                    addLogEntry(`failed: ${data.message}`, 'error');
                    diagStatus.textContent = 'ERROR';
                }
            } catch (error) {
                console.error('Error sending command:', error);
                addLogEntry(`system error: ${error.message}`, 'error');
                diagStatus.textContent = 'ERROR';
            }
        }
        
        // Mock image upload
        async function uploadMockImage(event) {
            const file = event.target.files[0];
            if (!file) return;
            
            addLogEntry(`uploading: ${file.name}`);
            
            // Display image in camera feed
            const reader = new FileReader();
            reader.onload = function(e) {
                cameraFeed.innerHTML = `<img src="${e.target.result}" alt="Mock camera feed">`;
                addLogEntry('mock image loaded');
            };
            reader.readAsDataURL(file);
            
            // Upload to backend
            const formData = new FormData();
            formData.append('image', file);
            formData.append('camera', 'front');
            
            try {
                const response = await fetch('/api/mock/image', {
                    method: 'POST',
                    body: formData
                });
                
                const data = await response.json();
                
                if (data.success) {
                    addLogEntry(`uploaded: ${data.filename}`);
                } else {
                    addLogEntry(`upload failed: ${data.message}`, 'error');
                }
            } catch (error) {
                console.error('Error uploading image:', error);
                addLogEntry(`upload error: ${error.message}`, 'error');
            }
        }
        
        // Initialize
        connectWebSocket();
        addLogEntry('system initialized');
    </script>
</body>
</html>
        """

    async def broadcast(self, message: str):
        """Broadcast message to all connected WebSocket clients."""
        if not self.active_connections:
            return

        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                self.logger.warning(f"Failed to send to WebSocket client: {e}")
                disconnected.append(connection)

        # Remove disconnected clients
        for conn in disconnected:
            if conn in self.active_connections:
                self.active_connections.remove(conn)

    def broadcast_sync(self, message: str):
        """Synchronous wrapper for broadcast (for use from non-async code)."""
        if self.active_connections:
            # Get or create event loop
            try:
                loop = asyncio.get_running_loop()
                # Already in event loop, use create_task
                asyncio.create_task(self.broadcast(message))
            except RuntimeError:
                # No event loop, create one
                asyncio.run(self.broadcast(message))

    def get_mock_image(self, camera: str = "front") -> Optional[bytes]:
        """Get mock image data for specified camera.
        
        This method can be called by vision skills to retrieve uploaded mock images
        for testing without robot hardware.
        
        Args:
            camera: Camera name ('front', 'left', 'right')
            
        Returns:
            Image data as bytes, or None if no mock image available
        """
        return self.mock_images.get(camera)

    def has_mock_image(self, camera: str = "front") -> bool:
        """Check if mock image exists for specified camera."""
        return camera in self.mock_images

    def start(self):
        """Start web server in background thread."""

        def run_server():
            config = uvicorn.Config(
                self.app, host=self.host, port=self.port, log_level="info"
            )
            self.server = uvicorn.Server(config)
            self.server.run()

        self.server_thread = threading.Thread(
            target=run_server, daemon=True, name="WebInterface"
        )
        self.server_thread.start()

        self.logger.info(f"🌐 Web interface starting on http://{self.host}:{self.port}")
        self.logger.info(f"   Dashboard: http://localhost:{self.port}/")
        self.logger.info(f"   API: http://localhost:{self.port}/api/mission")
        self.logger.info(f"   WebSocket: ws://localhost:{self.port}/ws/status")

    def stop(self):
        """Stop web server."""
        if self.server:
            self.server.should_exit = True
            self.logger.info("Web interface stopped")


# Example standalone usage
if __name__ == "__main__":
    # Example callback
    def handle_command(command: str) -> dict:
        print(f"Received command: {command}")
        return {"success": True, "message": f"Executed: {command}"}

    # Create and start web interface
    web = WebInterface(command_callback=handle_command, port=8080)
    web.start()

    print("Web interface running on http://localhost:8080")
    print("Press Ctrl+C to stop")

    try:
        import time

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
        web.stop()
