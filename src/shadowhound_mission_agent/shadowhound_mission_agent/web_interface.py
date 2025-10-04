#!/usr/bin/env python3
"""Web interface for ShadowHound mission control.

Provides a FastAPI-based web server that can be embedded in any ROS2 node.
Designed to be reusable and independent of specific node implementation.
"""

from typing import Callable, Optional, Dict, Any
import asyncio
import threading
import logging
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
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
        logger: Optional[logging.Logger] = None
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
        
        # Server state
        self.server = None
        self.server_thread = None
        self.app = self._create_app()
        
    def _create_app(self) -> FastAPI:
        """Create FastAPI application with routes."""
        app = FastAPI(
            title="ShadowHound Mission Control",
            description="Web interface for autonomous robot control",
            version="0.1.0"
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
                    command=cmd.command
                )
            except Exception as e:
                self.logger.error(f"Error executing command: {e}")
                return MissionResponse(
                    success=False,
                    message=str(e),
                    command=cmd.command
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
    <title>🐕 ShadowHound Mission Control</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        .header {
            text-align: center;
            color: white;
            margin-bottom: 30px;
        }
        .header h1 {
            font-size: 3em;
            margin-bottom: 10px;
        }
        .card {
            background: white;
            border-radius: 15px;
            padding: 30px;
            margin-bottom: 20px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        .status-box {
            background: #f8f9fa;
            border-left: 4px solid #667eea;
            padding: 20px;
            border-radius: 5px;
            margin: 20px 0;
            font-family: 'Monaco', 'Courier New', monospace;
            min-height: 100px;
        }
        .status-box .status-text {
            color: #333;
            font-size: 1.1em;
        }
        .command-input {
            width: 100%;
            padding: 15px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 1.1em;
            transition: border-color 0.3s;
        }
        .command-input:focus {
            outline: none;
            border-color: #667eea;
        }
        .btn {
            padding: 15px 30px;
            border: none;
            border-radius: 8px;
            font-size: 1.1em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s;
            margin: 5px;
        }
        .btn-primary {
            background: #667eea;
            color: white;
        }
        .btn-primary:hover {
            background: #5568d3;
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
        }
        .btn-secondary {
            background: #6c757d;
            color: white;
        }
        .btn-secondary:hover {
            background: #5a6268;
        }
        .quick-commands {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 10px;
            margin-top: 20px;
        }
        .connection-status {
            display: inline-block;
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 0.9em;
            font-weight: 600;
        }
        .connected {
            background: #d4edda;
            color: #155724;
        }
        .disconnected {
            background: #f8d7da;
            color: #721c24;
        }
        h2 {
            color: #333;
            margin-bottom: 20px;
            font-size: 1.8em;
        }
        h3 {
            color: #555;
            margin-bottom: 15px;
            font-size: 1.3em;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🐕 ShadowHound</h1>
            <p>Autonomous Robot Mission Control</p>
            <span id="wsStatus" class="connection-status disconnected">Connecting...</span>
        </div>
        
        <div class="card">
            <h2>📊 Mission Status</h2>
            <div class="status-box">
                <div id="status" class="status-text">Waiting for updates...</div>
            </div>
        </div>
        
        <div class="card">
            <h2>🎮 Command Center</h2>
            <h3>Custom Command</h3>
            <input 
                type="text" 
                id="commandInput" 
                class="command-input" 
                placeholder="Enter mission command (e.g., 'stand up and wave hello')"
                onkeypress="if(event.key === 'Enter') sendCommand()"
            >
            <button class="btn btn-primary" onclick="sendCommand()">▶️ Execute Command</button>
            
            <h3 style="margin-top: 30px;">Quick Commands</h3>
            <div class="quick-commands">
                <button class="btn btn-secondary" onclick="sendQuick('stand up')">🧍 Stand Up</button>
                <button class="btn btn-secondary" onclick="sendQuick('sit down')">🪑 Sit Down</button>
                <button class="btn btn-secondary" onclick="sendQuick('wave hello')">👋 Wave</button>
                <button class="btn btn-secondary" onclick="sendQuick('perform dance 1')">💃 Dance</button>
                <button class="btn btn-secondary" onclick="sendQuick('stretch')">🤸 Stretch</button>
                <button class="btn btn-secondary" onclick="sendQuick('balance stand')">⚖️ Balance</button>
            </div>
        </div>
    </div>
    
    <script>
        let ws;
        const statusDiv = document.getElementById('status');
        const wsStatusSpan = document.getElementById('wsStatus');
        const commandInput = document.getElementById('commandInput');
        
        // WebSocket connection
        function connectWebSocket() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            ws = new WebSocket(`${protocol}//${window.location.host}/ws/status`);
            
            ws.onopen = () => {
                console.log('WebSocket connected');
                wsStatusSpan.textContent = 'Connected';
                wsStatusSpan.className = 'connection-status connected';
            };
            
            ws.onmessage = (event) => {
                console.log('Status update:', event.data);
                statusDiv.textContent = event.data;
                statusDiv.style.color = event.data.includes('FAILED') ? '#dc3545' : 
                                      event.data.includes('COMPLETED') ? '#28a745' : '#333';
            };
            
            ws.onclose = () => {
                console.log('WebSocket disconnected');
                wsStatusSpan.textContent = 'Disconnected';
                wsStatusSpan.className = 'connection-status disconnected';
                // Reconnect after 3 seconds
                setTimeout(connectWebSocket, 3000);
            };
            
            ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };
        }
        
        // Send command via REST API
        async function sendCommand() {
            const command = commandInput.value.trim();
            if (!command) {
                alert('Please enter a command');
                return;
            }
            
            sendQuick(command);
            commandInput.value = '';
        }
        
        async function sendQuick(command) {
            statusDiv.textContent = `⏳ Sending: ${command}`;
            statusDiv.style.color = '#666';
            
            try {
                const response = await fetch('/api/mission', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({command: command})
                });
                
                const data = await response.json();
                console.log('Response:', data);
                
                if (data.success) {
                    statusDiv.textContent = `✅ ${data.message}`;
                    statusDiv.style.color = '#28a745';
                } else {
                    statusDiv.textContent = `❌ ${data.message}`;
                    statusDiv.style.color = '#dc3545';
                }
            } catch (error) {
                console.error('Error sending command:', error);
                statusDiv.textContent = `❌ Error: ${error.message}`;
                statusDiv.style.color = '#dc3545';
            }
        }
        
        // Initialize
        connectWebSocket();
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
            asyncio.run(self.broadcast(message))
    
    def start(self):
        """Start web server in background thread."""
        def run_server():
            config = uvicorn.Config(
                self.app,
                host=self.host,
                port=self.port,
                log_level="info"
            )
            self.server = uvicorn.Server(config)
            self.server.run()
        
        self.server_thread = threading.Thread(
            target=run_server,
            daemon=True,
            name="WebInterface"
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
        return {
            "success": True,
            "message": f"Executed: {command}"
        }
    
    # Create and start web interface
    web = WebInterface(
        command_callback=handle_command,
        port=8080
    )
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
