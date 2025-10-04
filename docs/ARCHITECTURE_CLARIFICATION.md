# Architecture Clarification: Where Does the DIMOS Agent Live?

**TL;DR**: The DIMOS agent is a **separate Python object** that the ROS node **wraps around**. Think of the ROS node as a thin API gateway.

---

## Current Architecture (Layer by Layer)

### Layer Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     Layer 1: ROS2 Node                          │
│  (MissionAgentNode - Our thin wrapper)                         │
│                                                                  │
│  Responsibilities:                                              │
│  - ROS2 lifecycle (start/stop)                                 │
│  - Topic subscription (/mission_command)                       │
│  - Topic publishing (/mission_status)                          │
│  - Parameter management (agent_backend, mock_robot, etc.)      │
│  - Convert ROS messages → Python strings                       │
│  - Convert Python results → ROS messages                       │
│                                                                  │
│  Does NOT:                                                      │
│  - Process natural language                                     │
│  - Execute skills                                               │
│  - Control robot                                                │
│  - Store memory/context                                         │
└────────────────┬────────────────────────────────────────────────┘
                 │ Python function calls
                 │ (not ROS messages)
┌────────────────▼────────────────────────────────────────────────┐
│                Layer 2: DIMOS Agent                             │
│  (OpenAIAgent or PlanningAgent - Pure Python objects)          │
│                                                                  │
│  Responsibilities:                                              │
│  - Natural language understanding (via LLM)                     │
│  - Mission planning and decomposition                           │
│  - Skill selection and sequencing                               │
│  - Semantic memory (ChromaDB)                                   │
│  - Tool/function calling                                        │
│  - Context management                                           │
│                                                                  │
│  Does NOT:                                                      │
│  - Know about ROS2 at all                                       │
│  - Subscribe to topics                                          │
│  - Publish messages                                             │
└────────────────┬────────────────────────────────────────────────┘
                 │ Python function calls
                 │
┌────────────────▼────────────────────────────────────────────────┐
│             Layer 3: DIMOS Skills                               │
│  (UnitreeSkills - Pure Python objects)                         │
│                                                                  │
│  Responsibilities:                                              │
│  - Skill registry (40+ behaviors)                               │
│  - Skill execution                                              │
│  - Safety validation                                            │
│  - Result reporting                                             │
│                                                                  │
│  Does NOT:                                                      │
│  - Know about ROS2                                              │
│  - Know about LLMs                                              │
└────────────────┬────────────────────────────────────────────────┘
                 │ Python function calls
                 │
┌────────────────▼────────────────────────────────────────────────┐
│            Layer 4: Robot Interface                             │
│  (UnitreeGo2 + UnitreeROSControl)                              │
│                                                                  │
│  Responsibilities:                                              │
│  - Hardware abstraction                                         │
│  - ROS2 topics for robot (DIMOS manages these)                 │
│  - Mock/real robot switching                                    │
│  - Low-level control                                            │
└─────────────────────────────────────────────────────────────────┘
```

---

## Key Insight: The ROS Node is Just a Wrapper

### What Our ROS Node Does

```python
class MissionAgentNode(Node):  # <-- ROS2 Node (our code)
    def __init__(self):
        # 1. ROS2 setup
        super().__init__("shadowhound_mission_agent")
        
        # 2. Create DIMOS agent as a Python object
        self.agent = OpenAIAgent(robot, skills)  # <-- Pure Python, not ROS
        
        # 3. Create ROS interfaces
        self.sub = self.create_subscription(...)
        self.pub = self.create_publisher(...)
    
    def mission_callback(self, msg):
        """Bridge: ROS → DIMOS → ROS"""
        # 1. Receive ROS message
        command = msg.data  # Extract string
        
        # 2. Call DIMOS agent (pure Python)
        result = self.agent.process_text(command)  # <-- No ROS here!
        
        # 3. Publish result back to ROS
        status_msg = String()
        status_msg.data = f"COMPLETED: {result}"
        self.pub.publish(status_msg)
```

### What Lives Where

| Component | Lives In | Type | Knows About ROS? |
|-----------|----------|------|------------------|
| `MissionAgentNode` | ROS2 process | ROS Node | ✅ Yes - that's its job |
| `OpenAIAgent` | Python object | DIMOS class | ❌ No |
| `PlanningAgent` | Python object | DIMOS class | ❌ No |
| `UnitreeSkills` | Python object | DIMOS class | ❌ No |
| `UnitreeGo2` | Python object | DIMOS class | ❌ No (but UnitreeROSControl does) |

---

## Why This Architecture?

### 1. **Separation of Concerns**

```python
# ROS Node: "I speak ROS2"
- Knows: Topics, parameters, services, actions
- Doesn't care: How natural language works, what LLM does

# DIMOS Agent: "I speak natural language and skills"
- Knows: LLM prompts, skill selection, planning
- Doesn't care: How messages are transported
```

### 2. **Reusability**

The DIMOS agent can be used:
- ✅ In ROS2 (our use case)
- ✅ In a web server (FastAPI)
- ✅ In a CLI tool
- ✅ In a Jupyter notebook
- ✅ In a Discord bot

**Because it doesn't know about ROS2 at all!**

### 3. **Testing**

```python
# Test DIMOS agent without ROS
agent = OpenAIAgent(robot, skills)
result = agent.process_text("stand up")
assert result.success

# Test ROS node without agent (mock)
node = MissionAgentNode()
# Mock the agent
node.agent = MockAgent()
```

---

## Object Relationships (Code View)

### Creation Flow

```python
# In mission_agent.py

# Step 1: Create ROS node
class MissionAgentNode(Node):
    def __init__(self):
        super().__init__("shadowhound_mission_agent")  # ROS2 node
        
        # Step 2: Create robot interface (DIMOS, not our code)
        ros_control = UnitreeROSControl(mock_connection=True)
        self.robot = UnitreeGo2(ros_control=ros_control)
        
        # Step 3: Create skills (DIMOS, not our code)
        self.skills = UnitreeSkills(robot=self.robot)
        
        # Step 4: Create agent (DIMOS, not our code)
        self.agent = OpenAIAgent(
            robot=self.robot,
            skills=self.skills
        )
        
        # Step 5: Create ROS interfaces (our code)
        self.sub = self.create_subscription(...)
        self.pub = self.create_publisher(...)
```

### Ownership Tree

```
MissionAgentNode (ROS Node - OUR CODE)
├── self.robot (DIMOS Python object)
├── self.skills (DIMOS Python object)
├── self.agent (DIMOS Python object)
├── self.mission_sub (ROS subscriber - OUR CODE)
└── self.status_pub (ROS publisher - OUR CODE)
```

### Communication Flow

```
External World → ROS Topic → Our Node → DIMOS Agent → Skills → Robot
                                ↓
                    Python function call (NOT ROS!)
```

---

## Comparison: What If Agent WAS the ROS Node?

### Hypothetical Bad Design

```python
# If we made DIMOS agent inherit from Node (DON'T DO THIS)
class BadOpenAIAgent(Node, OpenAIAgent):  # ❌ Tight coupling
    def __init__(self):
        Node.__init__(self, "agent")
        OpenAIAgent.__init__(self, robot, skills)
        
        # Now agent is tied to ROS!
        # Can't use in web server, CLI, etc.
        # Harder to test
        # DIMOS would need to know about ROS
```

### Our Good Design

```python
# What we actually do (GOOD)
class MissionAgentNode(Node):  # ROS2 adapter
    def __init__(self):
        super().__init__("shadowhound_mission_agent")
        
        # Compose with DIMOS agent (not inherit)
        self.agent = OpenAIAgent(...)  # ✅ Pure Python, reusable
        
        # Bridge between ROS and agent
        self.sub = self.create_subscription(...)
```

---

## Web Interface Implications

### Option 1: Add Web to ROS Node (What We Discussed)

```python
class MissionAgentNode(Node):
    def __init__(self):
        # ROS setup
        self.agent = OpenAIAgent(...)
        
        # Add web server
        self.web = RobotWebInterface(...)
        
        # Both ROS and web call the SAME agent
        @self.web.app.post("/mission")
        async def web_command(cmd):
            return self.agent.process_text(cmd)  # Same agent!
        
        def ros_callback(self, msg):
            return self.agent.process_text(msg.data)  # Same agent!
```

**Advantage**: Single process, single agent instance, shared state

### Option 2: Separate Web Node

```python
# Node 1: ROS interface
class MissionAgentNode(Node):
    self.agent = OpenAIAgent(...)  # Agent instance 1

# Node 2: Web interface
class WebNode(Node):
    self.agent = OpenAIAgent(...)  # Agent instance 2 (different!)
```

**Disadvantage**: Two agent instances, no shared memory/context

### Option 3: Agent as Standalone Service

```python
# Standalone Python service (NO ROS)
agent = OpenAIAgent(...)

# ROS node calls it
class RosNode(Node):
    def callback(self, msg):
        result = requests.post("http://localhost:8000/mission", 
                              json={"cmd": msg.data})

# Web server calls it
@app.post("/mission")
def mission(cmd):
    return agent.process_text(cmd)
```

**Advantage**: Truly decoupled, but adds HTTP overhead

---

## Summary

### Question: "Does the agent live inside the ROS node?"

**Answer**: 

✅ **YES** - The agent *object* lives inside the node's Python process  
❌ **NO** - The agent is *not* a ROS node itself, just a Python object

**Analogy**:
```
ROS Node = A house
DIMOS Agent = A person living in the house

The person (agent) doesn't need to know about:
- The doorbell (ROS topics)
- The mailbox (ROS services)
- The address (node name)

The house (ROS node) handles all that, and just tells the person (agent):
"Someone sent you a message: 'stand up'"

The person (agent) does their thing, then tells the house (node):
"Tell them I'm done standing"

The house (node) sends that via the mailbox (ROS topic)
```

### Why This Matters for Web Interface

**If we add web to the ROS node**:
- Same process
- Same agent instance
- Shared memory/context between ROS and web

**If we create separate web node**:
- Two processes
- Two agent instances
- Need to coordinate via ROS topics

**Recommendation**: Add web server to same ROS node (Option 1) because:
1. Share agent instance and context
2. Simpler deployment (one process)
3. No inter-process communication overhead
4. Agent doesn't care if command came from ROS or web

---

Does this clarify the architecture? The key is: **The agent is a tool the ROS node uses, not part of ROS itself.**
