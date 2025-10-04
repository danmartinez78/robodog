"""Example: Adding RAG memory to ShadowHound mission agent.

This example shows how to create a custom knowledge base for your robot
using DIMOS's semantic memory (RAG) capabilities.

Usage:
    1. Copy relevant parts into mission_agent.py
    2. Customize knowledge_base list with your data
    3. Agent will automatically use RAG for all queries
"""

from dimos.agents.memory.chroma_impl import OpenAISemanticMemory, LocalSemanticMemory
from datetime import datetime
import os


class MissionMemoryManager:
    """Helper class to manage mission history and robot knowledge."""
    
    def __init__(self, use_local_embeddings=False):
        """Initialize memory manager.
        
        Args:
            use_local_embeddings: If True, use free local embeddings.
                                  If False, use OpenAI embeddings (costs $).
        """
        if use_local_embeddings:
            self.memory = LocalSemanticMemory(
                collection_name="shadowhound_missions",
                model_name="sentence-transformers/all-MiniLM-L6-v2"
            )
        else:
            self.memory = OpenAISemanticMemory(
                collection_name="shadowhound_missions",
                model="text-embedding-3-large",
                dimensions=1024
            )
    
    def load_robot_knowledge(self):
        """Load static robot knowledge into RAG database."""
        
        knowledge_base = [
            # Robot Capabilities
            {
                "id": "capability_movement",
                "content": (
                    "Unitree Go2 Movement Capabilities: "
                    "Walking speed: 0-1.6 m/s, Trotting speed: 0-3.5 m/s. "
                    "Can climb slopes up to 45 degrees. "
                    "Can step over obstacles up to 20cm height. "
                    "Can climb stairs with steps up to 18cm. "
                    "Maximum safe indoor speed: 1.5 m/s. "
                    "IP67 waterproof rating (can handle light rain)."
                )
            },
            {
                "id": "capability_sensors",
                "content": (
                    "Unitree Go2 Sensor Suite: "
                    "5x RGB cameras (front, sides, rear) for 360° vision. "
                    "1x 3D LiDAR for obstacle detection up to 10m. "
                    "IMU (Inertial Measurement Unit) for balance and orientation. "
                    "Proprioceptive joint sensors on all 12 joints. "
                    "Optional thermal camera for heat detection. "
                    "GPS with ±2m accuracy for outdoor navigation."
                )
            },
            {
                "id": "capability_battery",
                "content": (
                    "Unitree Go2 Battery Specifications: "
                    "Capacity: 8000mAh, 29.6V lithium-ion. "
                    "Runtime: 1-2 hours depending on terrain and speed. "
                    "Charge time: 90 minutes from empty to full. "
                    "Operating temperature: -20°C to 55°C. "
                    "Battery management system with cell balancing."
                )
            },
            
            # Safety Protocols
            {
                "id": "safety_battery",
                "content": (
                    "Battery Safety Protocol: "
                    "Minimum 20% charge required to start any mission. "
                    "Automatic return-to-base triggered at 15% charge. "
                    "Low battery warning issued at 25% charge. "
                    "Never operate below 10% charge as it may damage battery. "
                    "Monitor battery temperature - abort if >50°C."
                )
            },
            {
                "id": "safety_terrain",
                "content": (
                    "Terrain Navigation Safety: "
                    "Avoid water deeper than 3cm (splash-proof, not waterproof). "
                    "Do not attempt slopes steeper than 45 degrees. "
                    "Avoid gaps wider than 30cm between surfaces. "
                    "Use slow mode (walk) in crowded or confined spaces. "
                    "Always test new terrain types with supervision first."
                )
            },
            {
                "id": "safety_emergency",
                "content": (
                    "Emergency Procedures: "
                    "Emergency stop: Press red button on robot's back. "
                    "Lost connection: Robot will sit down and wait for reconnection. "
                    "If robot falls: Self-recovery mode will attempt to stand. "
                    "If stuck: Remote operator can take manual control. "
                    "In case of fire/smoke: Immediate evacuation of robot. "
                    "Contact operator if any unusual sounds or vibrations detected."
                )
            },
            
            # Mission Types
            {
                "id": "mission_patrol",
                "content": (
                    "Patrol Mission Guidelines: "
                    "Standard patrol speed: 1.0 m/s on flat terrain. "
                    "Patrol routes should be pre-mapped and tested. "
                    "Stop and inspect any detected anomalies. "
                    "Report all findings to operator via status messages. "
                    "Complete perimeter check every 30 minutes. "
                    "Return to starting position after patrol completion."
                )
            },
            {
                "id": "mission_inspection",
                "content": (
                    "Inspection Mission Guidelines: "
                    "Approach inspection target slowly (0.5 m/s). "
                    "Capture images from multiple angles (front, sides, top if possible). "
                    "Use thermal camera if temperature anomalies suspected. "
                    "Maintain 1m distance from delicate equipment. "
                    "Document all observations in mission log. "
                    "Complete inspection checklist before marking mission complete."
                )
            },
            {
                "id": "mission_delivery",
                "content": (
                    "Delivery Mission Guidelines: "
                    "Maximum payload: 3kg (distributed evenly). "
                    "Reduce speed to 0.8 m/s when carrying payload. "
                    "Avoid stairs when carrying fragile items. "
                    "Verify payload is secure before starting navigation. "
                    "Use smooth motion profile to prevent jostling. "
                    "Confirm delivery with recipient and capture photo proof."
                )
            },
            
            # Skills Reference
            {
                "id": "skills_movement",
                "content": (
                    "Available Movement Skills: "
                    "stand() - Stand up from sitting position. "
                    "sit() - Sit down from standing position. "
                    "walk(direction, speed) - Walk in specified direction. "
                    "rotate(angle) - Rotate in place by specified angle. "
                    "goto(x, y, yaw) - Navigate to specific coordinates. "
                    "follow_path(waypoints) - Follow predefined path."
                )
            },
            {
                "id": "skills_perception",
                "content": (
                    "Available Perception Skills: "
                    "capture_image(camera) - Capture image from specified camera. "
                    "detect_objects() - Detect objects in camera view. "
                    "detect_person() - Detect people in vicinity. "
                    "measure_distance(direction) - Measure distance using lidar. "
                    "scan_environment() - Create 3D map of surroundings. "
                    "thermal_scan() - Scan for thermal anomalies (if equipped)."
                )
            },
            {
                "id": "skills_interaction",
                "content": (
                    "Available Interaction Skills: "
                    "wave(leg) - Wave specified leg in greeting. "
                    "dance(style) - Perform dance routine. "
                    "speak(text) - Text-to-speech output through speakers. "
                    "display_emotion(emotion) - Show emotion on LED display. "
                    "play_sound(sound_id) - Play predefined sound effect. "
                    "gesture(gesture_type) - Perform specific gesture."
                )
            },
        ]
        
        # Add all knowledge to vector database
        for item in knowledge_base:
            self.memory.add_vector(item["id"], item["content"])
        
        return len(knowledge_base)
    
    def add_mission_log(self, mission_data):
        """Add a completed mission to the knowledge base.
        
        Args:
            mission_data: Dict with mission details
                {
                    "command": str,
                    "result": str,
                    "battery_start": int,
                    "battery_end": int,
                    "duration_sec": float,
                    "anomalies": list,
                    "notes": str
                }
        """
        mission_id = f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        content = (
            f"Mission executed at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}. "
            f"Command: {mission_data['command']}. "
            f"Result: {mission_data['result']}. "
            f"Battery: {mission_data['battery_start']}% → {mission_data['battery_end']}%. "
            f"Duration: {mission_data['duration_sec']:.1f} seconds. "
        )
        
        if mission_data.get('anomalies'):
            content += f"Anomalies detected: {', '.join(mission_data['anomalies'])}. "
        
        if mission_data.get('notes'):
            content += f"Notes: {mission_data['notes']}"
        
        self.memory.add_vector(mission_id, content)
        return mission_id
    
    def search(self, query, n_results=3, threshold=0.5):
        """Search the knowledge base.
        
        Args:
            query: Search query string
            n_results: Number of results to return
            threshold: Minimum similarity score (0-1)
            
        Returns:
            List of (document, score) tuples
        """
        return self.memory.query(
            query,
            n_results=n_results,
            similarity_threshold=threshold
        )


# =============================================================================
# INTEGRATION EXAMPLE: Add to mission_agent.py
# =============================================================================

def _init_agent_with_rag(self):
    """Example: Initialize agent with RAG memory.
    
    Add this to your MissionAgentNode class in mission_agent.py
    """
    
    # Create memory manager
    use_local = os.getenv("USE_LOCAL_EMBEDDINGS", "false").lower() == "true"
    self.memory_manager = MissionMemoryManager(use_local_embeddings=use_local)
    
    # Load robot knowledge
    self.get_logger().info("Loading robot knowledge base...")
    num_docs = self.memory_manager.load_robot_knowledge()
    self.get_logger().info(f"Loaded {num_docs} documents into RAG memory")
    
    # Initialize robot and skills (your existing code)
    ros_control = UnitreeROSControl(mock_connection=self.mock_robot)
    self.robot = UnitreeGo2(ros_control=ros_control, disable_video_stream=True)
    self.skills = UnitreeSkills(robot=self.robot)
    
    # Create agent WITH RAG memory
    from dimos.agents.agent import OpenAIAgent
    
    self.agent = OpenAIAgent(
        robot=self.robot,
        dev_name="shadowhound",
        agent_type="Mission",
        skills=self.skills,
        agent_memory=self.memory_manager.memory,  # ← RAG database!
        rag_query_n=3,                             # Retrieve top 3 docs
        rag_similarity_threshold=0.5,              # Moderate filtering
        model_name="gpt-4o",
    )
    
    self.get_logger().info("Agent initialized with RAG capabilities ✓")


def _mission_callback_with_logging(self, msg):
    """Example: Mission callback that logs to RAG memory.
    
    Add this to your MissionAgentNode class
    """
    command = msg.data
    start_time = datetime.now()
    
    # Your existing mission execution code
    try:
        result = self.agent.process_text(command)
        success = True
    except Exception as e:
        result = str(e)
        success = False
    
    # Calculate duration
    duration = (datetime.now() - start_time).total_seconds()
    
    # Log mission to RAG database
    if hasattr(self, 'memory_manager'):
        mission_data = {
            "command": command,
            "result": result,
            "battery_start": 85,  # Get from robot.get_battery()
            "battery_end": 80,    # Get from robot.get_battery()
            "duration_sec": duration,
            "anomalies": [],      # Get from mission results
            "notes": "Completed successfully" if success else "Failed"
        }
        
        mission_id = self.memory_manager.add_mission_log(mission_data)
        self.get_logger().info(f"Logged mission to RAG: {mission_id}")
    
    # Rest of your callback...


# =============================================================================
# TESTING EXAMPLE
# =============================================================================

def test_rag_memory():
    """Test RAG memory with sample queries."""
    
    # Create and populate memory
    memory_mgr = MissionMemoryManager(use_local_embeddings=True)
    num_docs = memory_mgr.load_robot_knowledge()
    print(f"✓ Loaded {num_docs} documents")
    
    # Test queries
    test_queries = [
        "Can the robot climb stairs?",
        "What should I do if battery is low?",
        "How do I perform a patrol mission?",
        "What sensors does the robot have?",
    ]
    
    for query in test_queries:
        print(f"\n🔍 Query: {query}")
        results = memory_mgr.search(query, n_results=2, threshold=0.3)
        
        for i, (doc, score) in enumerate(results, 1):
            print(f"  Result {i} (score: {score:.3f}):")
            print(f"    {doc.page_content[:100]}...")
    
    # Test adding mission log
    mission_data = {
        "command": "patrol the perimeter",
        "result": "Completed successfully",
        "battery_start": 100,
        "battery_end": 75,
        "duration_sec": 1800,
        "anomalies": ["Thermal signature in sector B"],
        "notes": "All sectors clear except minor anomaly"
    }
    
    mission_id = memory_mgr.add_mission_log(mission_data)
    print(f"\n✓ Added mission log: {mission_id}")
    
    # Query for the mission we just added
    print(f"\n🔍 Query: What was the last patrol?")
    results = memory_mgr.search("last patrol", n_results=1, threshold=0.2)
    if results:
        doc, score = results[0]
        print(f"  Found (score: {score:.3f}):")
        print(f"    {doc.page_content}")


if __name__ == "__main__":
    print("=" * 70)
    print("ShadowHound RAG Memory Example")
    print("=" * 70)
    
    # Check environment
    if not os.getenv("OPENAI_API_KEY"):
        print("\n⚠️  OPENAI_API_KEY not set - using local embeddings")
        os.environ["USE_LOCAL_EMBEDDINGS"] = "true"
    
    # Run test
    test_rag_memory()
    
    print("\n" + "=" * 70)
    print("✓ RAG memory test complete!")
    print("=" * 70)
