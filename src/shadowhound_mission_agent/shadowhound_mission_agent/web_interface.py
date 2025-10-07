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

        # Camera feed storage (latest frame)
        self.latest_camera_frame: Optional[bytes] = None

        # Diagnostics data
        self.diagnostics = {
            "robot_mode": "unknown",
            "topics_available": [],
            "action_servers": [],
            "last_update": None,
        }

        # Terminal buffer for command history
        self.terminal_buffer = []
        self.max_terminal_lines = 1000

        # Performance metrics storage (last N measurements)
        self.performance_metrics = []
        self.max_metrics = 50  # Keep last 50 measurements
        self.latest_timing = {
            "agent_duration": 0.0,
            "total_duration": 0.0,
            "overhead_duration": 0.0,
            "agent_percentage": 0.0,
            "avg_agent_duration": 0.0,
            "avg_total_duration": 0.0,
            "command_count": 0,
        }

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

                self.logger.info(
                    f"Mock image uploaded: {filename} ({len(image_data)} bytes)"
                )

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

        @app.get("/api/camera/latest")
        async def get_camera_frame():
            """Get latest camera frame as base64 encoded image."""
            if self.latest_camera_frame:
                b64_image = base64.b64encode(self.latest_camera_frame).decode("utf-8")
                return {"image": b64_image, "available": True}
            return {"image": None, "available": False}

        @app.get("/api/diagnostics")
        async def get_diagnostics():
            """Get robot diagnostics information."""
            return self.diagnostics

        @app.get("/api/terminal")
        async def get_terminal():
            """Get terminal buffer."""
            return {"lines": self.terminal_buffer[-100:]}

        @app.get("/api/performance")
        async def get_performance():
            """Get performance metrics."""
            return {
                "latest": self.latest_timing,
                "history": self.performance_metrics[-20:],  # Last 20 for graphing
            }

        return app

    def _get_dashboard_html(self) -> str:
        """Generate dashboard HTML."""
        # Read from template file
        template_path = Path(__file__).parent / "dashboard_template.html"
        if template_path.exists():
            return template_path.read_text()
        else:
            # Fallback to simple HTML if template not found
            return """
<!DOCTYPE html>
<html>
<head><title>ShadowHound</title></head>
<body>
    <h1>ShadowHound Mission Control</h1>
    <p>Template file not found. Please check dashboard_template.html</p>
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

    def update_camera_frame(self, image_data: bytes):
        """Update the latest camera frame.

        Args:
            image_data: JPEG or PNG image bytes
        """
        self.latest_camera_frame = image_data

    def update_diagnostics(self, diagnostics: Dict[str, Any]):
        """Update diagnostics information.

        Args:
            diagnostics: Dict containing robot_mode, topics_available, action_servers, etc.
        """
        import datetime

        self.diagnostics.update(diagnostics)
        self.diagnostics["last_update"] = datetime.datetime.now().isoformat()

    def add_terminal_line(self, line: str):
        """Add a line to the terminal buffer.

        Args:
            line: Text line to add
        """
        import datetime

        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.terminal_buffer.append(f"[{timestamp}] {line}")

        # Keep buffer at max size
        if len(self.terminal_buffer) > self.max_terminal_lines:
            self.terminal_buffer = self.terminal_buffer[-self.max_terminal_lines :]

    def update_performance_metrics(self, timing_info: Dict[str, Any]):
        """Update performance metrics with new timing information.

        Args:
            timing_info: Dict containing agent_duration, total_duration, overhead_duration
        """
        import datetime

        # Add timestamp to timing info
        timing_with_timestamp = timing_info.copy()
        timing_with_timestamp["timestamp"] = datetime.datetime.now().isoformat()

        # Add to history
        self.performance_metrics.append(timing_with_timestamp)

        # Keep only last N measurements
        if len(self.performance_metrics) > self.max_metrics:
            self.performance_metrics = self.performance_metrics[-self.max_metrics :]

        # Calculate averages
        if self.performance_metrics:
            avg_agent = sum(
                m["agent_duration"] for m in self.performance_metrics
            ) / len(self.performance_metrics)
            avg_total = sum(
                m["total_duration"] for m in self.performance_metrics
            ) / len(self.performance_metrics)
        else:
            avg_agent = 0.0
            avg_total = 0.0

        # Update latest timing with averages
        self.latest_timing = {
            **timing_info,
            "avg_agent_duration": avg_agent,
            "avg_total_duration": avg_total,
            "command_count": len(self.performance_metrics),
        }

        self.logger.info(
            f"Performance metrics updated: "
            f"Agent {timing_info['agent_duration']:.2f}s "
            f"(avg {avg_agent:.2f}s over {len(self.performance_metrics)} commands)"
        )

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
