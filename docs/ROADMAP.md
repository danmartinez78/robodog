# ShadowHound Development Roadmap

**Last Updated**: 2025-10-08  
**Branch**: dev

This document tracks planned features, investigations, and improvements for the ShadowHound project.

---

## 🔴 Critical Issues

### ROS Logging Warning Investigation
**Priority**: HIGH  
**Status**: 🔍 Investigating

```
[WARN] [rcl.logging_rosout]: Publisher already registered for provided node name.
If this is due to multiple nodes with the same name then all logs for that 
logger name will go out over the existing publisher. As soon as any node with 
that name is destructed it will unregister the publisher, preventing any further 
logs for that name from being published on the rosout topic.
```

**Context**: Appears during system startup. May indicate node name collision or lifecycle issue.

**Investigation Steps**:
- [ ] Identify which node(s) trigger the warning
- [ ] Check for duplicate node names in launch files
- [ ] Review node lifecycle initialization order
- [ ] Verify ROS logging configuration

---

### WebRTC Skills + Nav2 Interference
**Priority**: HIGH  
**Status**: 🔍 Investigating

**Problem**: 
- WebRTC skills (sit, stand) only work via CLI `ros2 topic pub`
- CLI publishing of WebRTC skills breaks Nav2 skills (including teleop)
- Joynode allows translation/rotation AND sit/stand without breaking either

**Hypothesis**: Potential command multiplexing or priority issue in the Unitree SDK bridge.

**Investigation Steps**:
- [ ] Analyze joynode implementation (how does it multiplex commands?)
- [ ] Review Unitree SDK bridge for command queue/priority handling
- [ ] Check if WebRTC and Nav2 commands use conflicting interfaces
- [ ] Test if issue is timing-related or architectural
- [ ] Consider implementing command arbiter pattern similar to joynode

**References**:
- Joynode allows both command types simultaneously without conflicts
- May need to implement proper command multiplexing layer

---

## 🚀 Feature Development

### Agent Personality System
**Priority**: MEDIUM  
**Status**: 💡 Planned

**Goal**: Add TARS-style personality to mission agent (humor setting, deadpan responses, etc.)

**Implementation Ideas**:
- Personality traits as LLM system prompt modifiers
- Configurable humor/sarcasm level (0-100%)
- Context-aware responses (mission-critical vs casual interaction)
- Voice personality parameters for TTS

**Example Traits**:
- Humor setting (like TARS: "Humor: 75%")
- Verbosity level
- Formality/casualness
- Emotional tone

**Tasks**:
- [ ] Design personality configuration schema
- [ ] Integrate personality prompts into LLM system message
- [ ] Add personality controls to web UI
- [ ] Create personality presets (TARS, HAL9000, helpful assistant, etc.)

---

### Voice Interface
**Priority**: MEDIUM  
**Status**: 💡 Planned

**Goals**:
- Agent can speak responses (TTS)
- Accept verbal commands (STT)
- Enable natural conversation mode

**Components**:
- **TTS (Text-to-Speech)**: 
  - Options: Coqui TTS, Azure TTS, Google TTS
  - Should support personality voice parameters
  - Low-latency requirement for responsive interaction
  
- **STT (Speech-to-Text)**:
  - Options: Whisper (local), Azure STT, Google STT
  - Wake word detection for conversation mode
  - Continuous vs push-to-talk modes

**Tasks**:
- [ ] Evaluate TTS options (latency, quality, offline capability)
- [ ] Evaluate STT options (accuracy, latency, local vs cloud)
- [ ] Design conversation state machine
- [ ] Integrate with mission agent
- [ ] Add audio I/O to Thor hardware stack
- [ ] Create voice skills (speak, listen, conversation_mode)

---

### RAG (Retrieval-Augmented Generation) Integration
**Priority**: MEDIUM  
**Status**: 💡 Planned

**Goals**:
- Enable agent to reference documentation, past missions, environment knowledge
- Improve agent responses with contextual memory
- Support long-term mission history

**Use Cases**:
- "What did we find in the lab last week?"
- "Show me the route we took yesterday"
- Reference robot capabilities from documentation
- Learn from past mission successes/failures

**Tasks**:
- [ ] Choose RAG framework (LangChain, LlamaIndex, custom)
- [ ] Design knowledge base schema (missions, maps, observations)
- [ ] Implement vector database (ChromaDB, Pinecone, local)
- [ ] Create ingestion pipeline for mission logs
- [ ] Integrate with DIMOS semantic memory (if compatible)
- [ ] Add RAG retrieval to agent decision loop

---

### DIMOS Semantic Memory Integration
**Priority**: MEDIUM  
**Status**: 🔍 Research Needed

**Goal**: Understand and integrate DIMOS perception/semantic memory capabilities

**Questions**:
- What is DIMOS semantic memory architecture?
- How does it differ from basic RAG?
- Can we leverage it for spatial reasoning?
- Integration points with current mission agent?

**Tasks**:
- [ ] Review DIMOS perception documentation
- [ ] Identify semantic memory APIs
- [ ] Design integration architecture
- [ ] Prototype semantic memory queries in missions
- [ ] Document findings and recommendations

---

## 🎨 Web UI Improvements

### Enhanced Camera Feed
**Priority**: MEDIUM  
**Status**: 💡 Planned

**Goal**: Improve web UI video stream quality and performance

**Current**: Uncompressed or inefficient streaming  
**Target**: Compressed feed (JPEG/H.264) with adaptive quality

**Tasks**:
- [ ] Switch to compressed image transport (compressed topics)
- [ ] Evaluate H.264 streaming (lower bandwidth)
- [ ] Add quality/FPS controls in web UI
- [ ] Implement adaptive bitrate based on network conditions
- [ ] Test bandwidth usage improvement

---

### LiDAR BEV (Bird's Eye View) Display
**Priority**: MEDIUM  
**Status**: 💡 Planned

**Goal**: Real-time 2D top-down LiDAR visualization in web UI

**Features**:
- Live 2D occupancy grid
- Robot position and orientation
- Nav2 costmap overlay (optional)
- Goal/waypoint markers

**Tasks**:
- [ ] Design BEV rendering (Canvas/WebGL)
- [ ] Subscribe to LiDAR scan or occupancy grid topics
- [ ] Implement coordinate transforms
- [ ] Add zoom/pan controls
- [ ] Show robot footprint and heading
- [ ] Add costmap overlay toggle

---

### Advanced Terminal
**Priority**: LOW  
**Status**: 💡 Planned

**Goal**: Full-featured terminal in web UI with built-in commands

**Features**:
- Command history
- Auto-completion
- Built-in shortcuts for common operations:
  - `skills` - List available skills
  - `status` - Robot status summary
  - `nodes` - ROS2 node list
  - `topics` - ROS2 topic list
  - `map` - Current map info
  - `teleop` - Quick teleop mode

**Tasks**:
- [ ] Research terminal libraries (xterm.js, etc.)
- [ ] Design command parser
- [ ] Implement common command shortcuts
- [ ] Add command help system
- [ ] Integrate with mission agent for context-aware commands

---

## 📚 Documentation Improvements

### Hardware Documentation Enhancement
**Priority**: HIGH  
**Status**: 📝 In Progress

**Goals**:
- Comprehensive network topology diagrams
- Power distribution diagrams
- Sensor integration guides
- Configuration examples

**Documents to Add**:
- [x] Network & Power Topologies (see attached docs)
- [x] Omni Vision Sensor Setup (see attached docs)
- [x] 360° Vision Exploration (see attached docs)
- [x] Power & Network Setup Guide (see attached docs)
- [ ] Integrate these docs into repository
- [ ] Add wiring photos/schematics
- [ ] Document Thor + GO2 + Router + Sensor configurations
- [ ] Create quick-reference setup guides

---

## 🤖 Skills Expansion

### Nav2 Skills
**Priority**: MEDIUM  
**Status**: 💡 Planned

**New Skills to Add**:
- `nav.set_goal_pose(x, y, yaw)` - Direct goal pose setting
- `nav.cancel_goal()` - Cancel current navigation
- `nav.get_status()` - Navigation status query
- `nav.set_speed(linear, angular)` - Speed limit adjustment
- `nav.follow_waypoints(waypoints)` - Multi-point navigation

---

### SLAM Toolbox Skills
**Priority**: MEDIUM  
**Status**: 💡 Planned

**New Skills to Add**:
- `slam.save_map(filename)` - Save current map
- `slam.load_map(filename)` - Load existing map
- `slam.start_mapping()` - Begin SLAM session
- `slam.stop_mapping()` - Finalize map
- `slam.get_map_quality()` - Map coverage/quality metrics

---

### ROS2 System Introspection Skills
**Priority**: MEDIUM  
**Status**: 💡 Planned

**New Skills to Add**:
- `ros.list_nodes()` - Get active nodes
- `ros.list_topics()` - Get available topics
- `ros.topic_hz(topic)` - Measure topic rate
- `ros.topic_echo(topic, count)` - Sample topic data
- `ros.list_actions()` - Get action servers
- `ros.list_services()` - Get available services
- `ros.node_info(node_name)` - Node details

**Use Case**: Enable agent to introspect system state and diagnose issues

---

## 🧠 Advanced AI Features

### NVIDIA Isaac Integration
**Priority**: LOW  
**Status**: 🔍 Research

**Goals**:
- Investigate NVIDIA Gr00t foundation models
- Evaluate Isaac for manipulation/navigation
- Understand integration with current stack

**Tasks**:
- [ ] Review NVIDIA Isaac Sim documentation
- [ ] Explore Gr00t foundation model capabilities
- [ ] Assess compatibility with Holoscan/ROS2 bridge
- [ ] Prototype simple Isaac integration
- [ ] Document findings and recommendations

**References**:
- NVIDIA Isaac Sim: https://developer.nvidia.com/isaac-sim
- Gr00t: https://developer.nvidia.com/blog/gr00t/

---

### NVIDIA ReMEmbR Integration
**Priority**: LOW  
**Status**: 🔍 Research

**Goal**: Investigate NVIDIA ReMEmbR for multimodal memory

**Tasks**:
- [ ] Research ReMEmbR architecture
- [ ] Evaluate use cases for robotics
- [ ] Compare with DIMOS semantic memory
- [ ] Assess integration complexity
- [ ] Document findings

---

## 🛡️ Safety & Monitoring

### ODD/COD Monitoring Integration
**Priority**: HIGH  
**Status**: 💡 Planned

**Goal**: Integrate Operational Design Domain (ODD) and Conditions of Operation (COD) monitoring

**Reference**: https://github.com/markomiz/open_odd

**Features**:
- Real-time ODD compliance monitoring
- Safety envelope tracking
- Degraded operation modes
- Automatic failsafe triggers

**Tasks**:
- [ ] Review open_odd repository
- [ ] Design ODD/COD definitions for ShadowHound
- [ ] Integrate monitoring nodes
- [ ] Add ODD status to diagnostics
- [ ] Implement safety responses to ODD violations
- [ ] Create ODD configuration for indoor/outdoor scenarios

---

## 📋 Project Organization

### Task Priorities
- 🔴 **Critical**: Blocking development or causes system instability
- 🟡 **High**: Important for next milestone
- 🟢 **Medium**: Valuable features for future releases
- ⚪ **Low**: Nice-to-have, research, exploration

### Status Labels
- 🔍 **Investigating**: Active research/debugging
- 💡 **Planned**: Design phase, not yet started
- 🚧 **In Progress**: Active development
- ✅ **Complete**: Finished and merged
- 📝 **Documentation**: Writing/updating docs
- ❌ **Blocked**: Waiting on dependency

---

## Contributing to Roadmap

To add items to the roadmap:
1. Create issue with detailed description
2. Tag with appropriate priority/status labels
3. Link to relevant documentation or research
4. Update this roadmap document

To update status:
1. Change status emoji and description
2. Add progress notes under task
3. Link to PRs or commits when work begins
4. Move to appropriate section when complete

---

*This roadmap is a living document. Priorities and timelines may shift based on project needs, discoveries, and user feedback.*
