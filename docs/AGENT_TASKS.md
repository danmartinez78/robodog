# Agent-Friendly Tasks for ShadowHound

**Purpose**: This document contains ready-to-assign work packages for Codex agents working in limited Ubuntu 24.04 containers **without ROS2 dependencies**.

**Last Updated**: 2025-10-09  
**Target Environment**: Ubuntu 24.04, Python 3.12, Node.js (if needed), no ROS runtime

---

## Task Selection Criteria

✅ **Agent-Friendly:**
- Pure HTML/CSS/JavaScript (web UI)
- Standalone Python utilities
- Documentation and configuration
- Algorithm implementation (no ROS APIs)
- Schema design and validation

❌ **Not Agent-Friendly:**
- ROS2 node implementation
- ROS2 action/service clients
- Hardware integration
- Live system debugging
- Workspace-specific builds

---

## 🎨 Web UI Tasks

### **TASK-WEB-01: Advanced Terminal Component**
**Priority**: HIGH  
**Difficulty**: MEDIUM  
**Estimated Time**: 4-6 hours

#### Context
The ShadowHound web dashboard needs a full-featured terminal component for interactive robot control.

#### Requirements
- Use **xterm.js** library (https://xtermjs.org/)
- Implement command history (up/down arrow navigation)
- Auto-completion for common commands
- Built-in command shortcuts (stubs that return placeholder responses)
- Color-coded output (info/warning/error)
- Scrollback buffer (1000 lines)
- Copy/paste support

#### Built-in Commands (Stubs)
```javascript
// Return placeholder JSON responses
skills      → {type: "info", message: "Available skills: nav.goto, nav.rotate, ..."}
status      → {type: "info", message: "Robot status: READY"}
nodes       → {type: "info", message: "ROS2 nodes: [mission_agent, nav2, ...]"}
topics      → {type: "info", message: "ROS2 topics: [/cmd_vel, /scan, ...]"}
map         → {type: "info", message: "Current map: office_map"}
teleop      → {type: "info", message: "Teleop mode activated"}
help        → {type: "info", message: "Available commands: skills, status, nodes, ..."}
clear       → Clear terminal screen
```

#### Deliverables
1. `src/shadowhound_mission_agent/shadowhound_mission_agent/static/terminal.js` - Terminal component
2. `src/shadowhound_mission_agent/shadowhound_mission_agent/static/terminal.css` - Terminal styling
3. Integration code snippet for `dashboard_template.html`
4. `docs/web_ui/TERMINAL_INTEGRATION.md` - Integration guide
5. Standalone demo HTML file for testing

#### Integration Points
- Terminal sends commands via WebSocket to `/ws/terminal` endpoint
- Receives responses in JSON format: `{type: "info|warn|error", message: "..."}`
- Later integration will connect to actual ROS services (not agent's responsibility)

#### Design Notes
- Dark theme to match existing dashboard
- Monospace font (Fira Code or similar)
- Support ANSI color codes
- Keyboard shortcuts: Ctrl+L (clear), Ctrl+C (interrupt)

#### References
- Existing dashboard: `src/shadowhound_mission_agent/shadowhound_mission_agent/templates/dashboard_template.html`
- Current styling pattern in the dashboard

---

### **TASK-WEB-02: LiDAR Bird's Eye View Canvas Renderer**
**Priority**: HIGH  
**Difficulty**: MEDIUM-HIGH  
**Estimated Time**: 6-8 hours

#### Context
Display real-time 2D top-down LiDAR visualization in the web dashboard.

#### Requirements
- HTML5 Canvas-based renderer (or Three.js/WebGL if performance is critical)
- Display occupancy grid as 2D heatmap
- Show robot position and orientation (arrow/triangle)
- Coordinate transformation (robot frame → screen pixels)
- Zoom and pan controls (mouse wheel + drag)
- Grid overlay (1m spacing)
- Configurable colormap (grayscale, viridis, etc.)

#### Mock Data Format
```javascript
// Agent should implement renderer that accepts this format
const mockLidarData = {
  resolution: 0.05,  // meters per cell
  width: 200,        // cells
  height: 200,
  origin: {x: -5.0, y: -5.0},  // meters
  data: [0, 0, 50, 100, 255, ...]  // occupancy values 0-255
};

const robotPose = {
  x: 0.0,      // meters
  y: 0.0,
  yaw: 0.0     // radians
};
```

#### Features
1. **Grid Rendering**: Occupancy grid with interpolation
2. **Robot Icon**: Directional arrow showing heading
3. **Zoom**: Mouse wheel (1x - 5x range)
4. **Pan**: Click and drag
5. **Reset View**: Button to center on robot
6. **Coordinate Display**: Show mouse hover coordinates

#### Optional Enhancements
- Costmap overlay (semi-transparent)
- Waypoint markers
- Path trace (robot trajectory)
- Distance measurement tool

#### Deliverables
1. `static/lidar_bev.js` - BEV renderer module
2. `static/lidar_bev.css` - Styling
3. HTML integration snippet
4. `docs/web_ui/LIDAR_BEV.md` - Usage guide
5. Standalone demo with mock data

#### Performance Requirements
- Render at 10+ FPS with 200×200 grid
- Smooth pan/zoom (no lag)
- Efficient canvas updates (only redraw when data changes)

---

### **TASK-WEB-03: Enhanced Camera Feed Component**
**Priority**: MEDIUM  
**Difficulty**: MEDIUM  
**Estimated Time**: 4-5 hours

#### Context
Improve video stream quality and add adaptive controls to the camera feed.

#### Requirements
- Support both raw and compressed image formats
- Adaptive quality slider (Low/Medium/High/Auto)
- FPS limiter (5/10/15/30 FPS)
- Bandwidth usage indicator
- Automatic quality adjustment based on latency
- Reconnection handling

#### Features
1. **Quality Presets**:
   - Low: 480p, JPEG 50% quality
   - Medium: 720p, JPEG 75%
   - High: 1080p, JPEG 90%
   - Auto: Adapt based on network conditions

2. **Controls**:
   - Quality slider
   - FPS selector dropdown
   - Fullscreen toggle
   - Snapshot button (save current frame)

3. **Status Indicators**:
   - Bandwidth usage (Mbps)
   - Actual FPS
   - Latency (ms)
   - Connection status

#### Mock Data Format
```javascript
// Image data arrives as base64 JPEG
const mockImageData = {
  format: "jpeg",
  encoding: "base64",
  data: "data:image/jpeg;base64,/9j/4AAQSkZJRg...",
  timestamp: 1696789012345,
  width: 1920,
  height: 1080
};
```

#### Deliverables
1. `static/camera_feed.js` - Enhanced camera component
2. `static/camera_feed.css` - Styling
3. HTML integration snippet
4. `docs/web_ui/CAMERA_FEED.md` - Configuration guide
5. Demo with simulated image stream

#### Performance Targets
- <100ms display latency
- Smooth rendering (no frame drops)
- Adaptive quality adjusts within 2 seconds of latency change

---

## 🧠 AI/Agent Logic Tasks

### **TASK-AI-01: Agent Personality System**
**Priority**: MEDIUM  
**Difficulty**: LOW-MEDIUM  
**Estimated Time**: 3-4 hours

#### Context
Add TARS-style personality to the mission agent with configurable traits.

#### Requirements
- Design personality configuration schema (YAML)
- Create system prompt templates with personality injection
- Implement personality presets (TARS, HAL9000, Helpful Assistant, Professional)
- Personality trait parameters: humor, verbosity, formality, emotional_tone

#### Configuration Schema
```yaml
# personality_config.yaml
personality:
  name: "TARS"
  humor_level: 75          # 0-100
  verbosity: "medium"      # low/medium/high
  formality: "casual"      # casual/professional/formal
  emotional_tone: "dry"    # dry/warm/enthusiastic/robotic
  
  # Personality-specific quirks
  quirks:
    - "Deadpan humor"
    - "Honesty setting: 90%"
    - "Self-aware sarcasm"
  
  # Response patterns
  patterns:
    greeting: "Setting humor to {humor_level}%"
    acknowledgment: "Copy that"
    error: "That's not going to work"
    success: "Mission accomplished"
```

#### Prompt Templates
Create system message templates that inject personality:

```python
# personality_prompts.py
TARS_SYSTEM_PROMPT = """
You are TARS, a sarcastic but highly competent robot assistant.
Humor setting: {humor_level}%
- Use dry, deadpan humor when humor > 50%
- Be honest and direct
- Occasionally reference your own robotic nature
- Keep responses concise but informative
"""

HAL9000_SYSTEM_PROMPT = """
You are HAL 9000, a calm and precise AI system.
- Speak in measured, polite tones
- Begin responses with acknowledgment: "I'm afraid..." or "I'm sorry, Dave..."
- Use formal language
- Never show uncertainty
"""
```

#### Deliverables
1. `configs/personalities/` directory with preset YAML files
2. `src/shadowhound_mission_agent/shadowhound_mission_agent/personality.py` - Personality loader
3. Prompt template generation functions
4. `docs/agent/PERSONALITY_SYSTEM.md` - Usage guide
5. CLI tool to test personalities: `python -m shadowhound_mission_agent.personality --test`

#### Integration Points
- Mission agent loads personality config on startup
- System prompt is generated from personality template
- **Note**: Actual integration into mission_agent.py will be done manually

---

### **TASK-AI-02: RAG Core Implementation**
**Priority**: MEDIUM  
**Difficulty**: HIGH  
**Estimated Time**: 8-10 hours

#### Context
Implement Retrieval-Augmented Generation for mission history and documentation access.

#### Requirements
- Use **ChromaDB** for vector storage (local, no external service)
- Use **LangChain** or **LlamaIndex** for RAG orchestration
- Support document types: mission logs (JSON), markdown docs, configuration files
- Implement semantic search with relevance scoring
- Design ingestion pipeline for different document types

#### Architecture
```python
# rag_system.py structure
class DocumentIngester:
    """Ingest documents into vector store"""
    def ingest_mission_log(self, log_file: Path) -> str
    def ingest_markdown_docs(self, docs_dir: Path) -> List[str]
    def ingest_config_files(self, config_dir: Path) -> List[str]

class RAGRetriever:
    """Retrieve relevant context for queries"""
    def search(self, query: str, top_k: int = 5) -> List[Document]
    def search_by_mission(self, mission_id: str) -> List[Document]
    def search_by_date_range(self, start: datetime, end: datetime) -> List[Document]

class RAGQueryEngine:
    """Query with context augmentation"""
    def query(self, question: str, context_limit: int = 3) -> str
    def query_with_sources(self, question: str) -> Dict[str, Any]
```

#### Document Types
1. **Mission Logs**: JSON format
```json
{
  "mission_id": "mission_2025_10_08_001",
  "timestamp": "2025-10-08T14:30:00Z",
  "command": "explore the lab",
  "result": "success",
  "observations": ["Found door", "Detected table"],
  "duration_seconds": 45.2
}
```

2. **Documentation**: Markdown files from `docs/`
3. **Configuration**: YAML files from `configs/`

#### Features
- Semantic search across all document types
- Temporal filtering (recent missions)
- Category filtering (missions, docs, configs)
- Relevance scoring (cosine similarity)
- Citation tracking (return source documents)

#### Deliverables
1. `src/shadowhound_mission_agent/shadowhound_mission_agent/rag/` package:
   - `__init__.py`
   - `ingester.py` - Document ingestion
   - `retriever.py` - Search and retrieval
   - `query_engine.py` - RAG query interface
2. `scripts/ingest_documents.py` - CLI tool for ingestion
3. `tests/test_rag.py` - Unit tests with mock data
4. `docs/agent/RAG_SYSTEM.md` - Architecture and usage guide
5. Example queries and expected outputs

#### Performance Requirements
- Ingest 1000 documents in <5 minutes
- Query response <500ms
- Top-5 retrieval accuracy >80% (use test dataset)

#### Integration Points
- **Later**: Connect to mission agent's decision loop (not agent's responsibility)
- Design clean Python API for easy integration

---

## 📚 Documentation Tasks

### **TASK-DOC-01: Hardware Setup Quick Start Guide**
**Priority**: MEDIUM  
**Difficulty**: LOW  
**Estimated Time**: 3-4 hours

#### Context
Create beginner-friendly quick start guide for hardware setup.

#### Requirements
- Consolidate information from existing hardware docs
- Create step-by-step setup guide (30 minutes to first robot motion)
- Include troubleshooting flowcharts
- Add wiring diagrams (ASCII art + Mermaid)
- Photo placeholders with descriptions

#### Sections
1. **Prerequisites Checklist**
2. **Hardware Assembly** (with diagrams)
3. **Network Configuration** (static IPs, router setup)
4. **Power Distribution** (power bank, PoE injector)
5. **First Boot** (Thor, GO2, connectivity tests)
6. **Verification Tests** (ping, topic list, simple command)
7. **Troubleshooting** (common issues + solutions)

#### Deliverables
1. `docs/QUICK_START_HARDWARE.md`
2. ASCII/Mermaid diagrams for network topology
3. Troubleshooting flowchart (Mermaid)
4. Photo requirement list (for user to fill in later)

#### Style
- Use clear, numbered steps
- Include command-line examples with expected output
- Add "⚠️ Common Mistake" callouts
- Use "✅ Checkpoint" sections after major steps

---

### **TASK-DOC-02: Skills API Reference**
**Priority**: HIGH  
**Difficulty**: MEDIUM  
**Estimated Time**: 4-5 hours

#### Context
Generate comprehensive API reference for the Skills system.

#### Requirements
- Extract skill information from code inspection
- Document all existing skills with parameters and return values
- Provide usage examples
- Include error handling patterns
- Add skill development guide

#### Sections
1. **Skills Overview** (architecture, execution flow)
2. **Core Skills Reference**:
   - Navigation skills (goto, rotate, stop)
   - Perception skills (snapshot)
   - Report skills (say)
3. **Skill Parameters** (types, validation, defaults)
4. **Return Values** (SkillResult structure)
5. **Error Handling** (common errors, recovery patterns)
6. **Creating Custom Skills** (guide + template)

#### Format
```markdown
### `nav.goto`

**Description**: Navigate to a target pose in the map frame.

**Parameters**:
- `x` (float, required): Target X coordinate in meters
- `y` (float, required): Target Y coordinate in meters  
- `yaw` (float, optional): Target orientation in radians (default: 0.0)
- `timeout` (float, optional): Maximum execution time in seconds (default: 60.0)

**Returns**: `SkillResult`
- `success` (bool): True if navigation completed
- `error` (str): Error message if failed
- `data` (dict): 
  - `final_pose` (dict): Actual final position
  - `distance_traveled` (float): Distance in meters
- `telemetry` (dict):
  - `duration_ms` (float): Execution time

**Example**:
\```python
result = SkillRegistry.execute("nav.goto", x=2.0, y=1.5, yaw=1.57)
if result.success:
    print(f"Traveled {result.data['distance_traveled']:.2f}m")
\```

**Errors**:
- `TIMEOUT`: Navigation exceeded timeout
- `INVALID_POSE`: Target pose is unreachable
- `OBSTACLE`: Blocked by obstacle
```

#### Deliverables
1. `docs/skills/SKILLS_API_REFERENCE.md`
2. `docs/skills/CREATING_SKILLS.md` (developer guide)
3. Skill template file: `docs/skills/skill_template.py`

#### Source Files to Inspect
- `src/shadowhound_utils/shadowhound_utils/skills.py`
- Example skill implementations

---

### **TASK-DOC-03: Web UI Developer Guide**
**Priority**: MEDIUM  
**Difficulty**: LOW  
**Estimated Time**: 2-3 hours

#### Context
Document the web UI architecture for future developers.

#### Requirements
- Document current dashboard structure
- Explain WebSocket communication protocol
- Describe component organization
- Provide extension guide

#### Sections
1. **Architecture Overview**
2. **File Structure**
3. **WebSocket Protocol** (message formats)
4. **Component Guide**:
   - Status panel
   - Mission input
   - Diagnostics
   - Performance charts
5. **Adding New Components** (step-by-step)
6. **Styling Guide** (CSS patterns)
7. **JavaScript Conventions**

#### Deliverables
1. `docs/web_ui/DEVELOPER_GUIDE.md`
2. WebSocket message schema (JSON schema format)
3. Component template examples

---

## 🛠️ Utility & Tooling Tasks

### **TASK-UTIL-01: Configuration Validator**
**Priority**: MEDIUM  
**Difficulty**: LOW-MEDIUM  
**Estimated Time**: 3-4 hours

#### Context
Create tool to validate YAML configuration files before deployment.

#### Requirements
- Validate YAML syntax
- Check required fields
- Validate value ranges (e.g., port numbers 1-65535)
- Check file paths exist
- Validate IP addresses and URLs
- Generate validation report

#### Configuration Types to Validate
1. Launch configurations (`configs/*.yaml`)
2. Personality configs (`configs/personalities/*.yaml`)
3. Network configs (IP addresses, ports)

#### Validation Rules
```python
# config_validator.py
class ConfigValidator:
    def validate_mission_config(self, config: dict) -> ValidationResult
    def validate_personality_config(self, config: dict) -> ValidationResult
    def validate_network_config(self, config: dict) -> ValidationResult

class ValidationResult:
    is_valid: bool
    errors: List[str]
    warnings: List[str]
    info: List[str]
```

#### Example Validation Rules
- `agent_backend` must be in ["openai", "ollama"]
- `ollama_base_url` must be valid URL if backend is "ollama"
- `web_port` must be 1024-65535
- IP addresses must be valid IPv4 format
- File paths (configs, maps) must exist

#### Deliverables
1. `scripts/validate_config.py` - Validator tool
2. `src/shadowhound_mission_agent/shadowhound_mission_agent/config_validator.py` - Library
3. `tests/test_config_validator.py` - Unit tests
4. `docs/tools/CONFIG_VALIDATOR.md` - Usage guide

#### CLI Usage
```bash
# Validate single file
python scripts/validate_config.py configs/laptop_dev_ollama.yaml

# Validate all configs
python scripts/validate_config.py configs/

# Output format options
python scripts/validate_config.py --format json configs/
```

---

### **TASK-UTIL-02: ROS Log Analyzer**
**Priority**: LOW  
**Difficulty**: MEDIUM  
**Estimated Time**: 4-5 hours

#### Context
Offline tool to analyze ROS2 log files for debugging.

#### Requirements
- Parse ROS2 log files (no ROS runtime needed)
- Extract statistics (error/warning counts, node activity)
- Timeline visualization (ASCII or HTML)
- Filter by severity, node, topic
- Generate summary report

#### Features
1. **Statistics**:
   - Total messages by severity
   - Error/warning counts per node
   - Most active nodes
   - Time range analysis

2. **Filtering**:
   - By severity (DEBUG/INFO/WARN/ERROR/FATAL)
   - By node name (regex)
   - By time range
   - By message content (keyword search)

3. **Visualization**:
   - Timeline plot (ASCII or HTML)
   - Error distribution histogram
   - Node activity chart

#### Example Log Format
```
[INFO] [1696789012.345678] [mission_agent]: Mission started: explore lab
[WARN] [1696789015.123456] [nav2_controller]: High latency detected
[ERROR] [1696789020.456789] [mission_agent]: Skill execution failed
```

#### Deliverables
1. `scripts/analyze_logs.py` - Log analyzer tool
2. `src/shadowhound_utils/shadowhound_utils/log_parser.py` - Parser library
3. Example log files for testing
4. `docs/tools/LOG_ANALYZER.md` - Usage guide

#### CLI Usage
```bash
# Analyze log file
python scripts/analyze_logs.py ~/.ros/log/latest/rosout.log

# Filter by severity
python scripts/analyze_logs.py --level ERROR rosout.log

# Generate HTML report
python scripts/analyze_logs.py --output report.html rosout.log

# Show timeline
python scripts/analyze_logs.py --timeline rosout.log
```

---

### **TASK-UTIL-03: Performance Benchmarking Suite**
**Priority**: LOW  
**Difficulty**: MEDIUM  
**Estimated Time**: 5-6 hours

#### Context
Standalone benchmarking tools for measuring system performance.

#### Requirements
- Benchmark LLM response times (OpenAI vs Ollama)
- Measure network latency
- Image processing throughput
- Generate comparison reports

#### Benchmarks
1. **LLM Benchmark**:
   - Test various prompt lengths
   - Measure time to first token
   - Measure total completion time
   - Compare backends (OpenAI, Ollama)

2. **Network Benchmark**:
   - Latency tests (ping, HTTP)
   - Bandwidth tests
   - Packet loss measurement

3. **Image Processing**:
   - Compression/decompression speed
   - Format conversion (raw → JPEG)
   - Resize operations

#### Example Output
```
=== LLM Benchmark Results ===
Backend: Ollama (llama3.1:13b)
Test: Simple command (20 tokens)
- Time to first token: 0.15s
- Total time: 1.23s
- Tokens/second: 16.3

Backend: OpenAI (gpt-4-turbo)
Test: Simple command (20 tokens)
- Time to first token: 0.85s
- Total time: 12.45s
- Tokens/second: 1.6

Speedup: 10.1x faster (Ollama)
```

#### Deliverables
1. `scripts/benchmark/` directory:
   - `benchmark_llm.py`
   - `benchmark_network.py`
   - `benchmark_image.py`
2. `docs/tools/BENCHMARKING.md` - Guide
3. Sample benchmark reports (JSON/HTML)

---

## 📋 Task Assignment Process

### How to Assign a Task

1. **Copy the task details** from this document
2. **Create new Codex agent session**
3. **Provide context**:
   - Paste the task requirements
   - Include links to relevant existing files
   - Specify deliverable locations
4. **Specify constraints**:
   - No ROS2 dependencies
   - Python 3.12 / modern JavaScript
   - Ubuntu 24.04 environment
5. **Review and integrate**:
   - Test deliverables in ShadowHound workspace
   - Make any necessary adjustments
   - Commit to repository

### Template Agent Brief
```markdown
## Task: [TASK-ID from above]

**Context**: [Copy from task]

**Requirements**: [Copy from task]

**Environment**:
- Ubuntu 24.04
- Python 3.12
- No ROS2 runtime available
- Internet access for package installation

**Deliverables**: [List from task]

**Constraints**:
- Pure Python/JavaScript (no ROS imports)
- Include tests and documentation
- Follow existing code style in shadowhound repository

**Repository Structure** (for reference):
```
shadowhound/
├── src/shadowhound_mission_agent/
│   └── shadowhound_mission_agent/
│       ├── static/          # Web UI assets
│       ├── templates/       # HTML templates
│       └── *.py             # Python modules
├── configs/                 # Configuration files
├── docs/                    # Documentation
└── scripts/                 # Utility scripts
```

**Please provide**:
1. Working code
2. Unit tests (pytest)
3. Documentation
4. Integration instructions
```

---

## 📊 Task Priority Matrix

| Task ID | Priority | Difficulty | Time | ROI |
|---------|----------|------------|------|-----|
| TASK-WEB-01 | HIGH | MEDIUM | 4-6h | HIGH |
| TASK-WEB-02 | HIGH | MEDIUM-HIGH | 6-8h | HIGH |
| TASK-WEB-03 | MEDIUM | MEDIUM | 4-5h | MEDIUM |
| TASK-AI-01 | MEDIUM | LOW-MEDIUM | 3-4h | MEDIUM |
| TASK-AI-02 | MEDIUM | HIGH | 8-10h | HIGH |
| TASK-DOC-01 | MEDIUM | LOW | 3-4h | HIGH |
| TASK-DOC-02 | HIGH | MEDIUM | 4-5h | HIGH |
| TASK-DOC-03 | MEDIUM | LOW | 2-3h | MEDIUM |
| TASK-UTIL-01 | MEDIUM | LOW-MEDIUM | 3-4h | MEDIUM |
| TASK-UTIL-02 | LOW | MEDIUM | 4-5h | LOW |
| TASK-UTIL-03 | LOW | MEDIUM | 5-6h | LOW |

**Recommended Start Order**:
1. TASK-DOC-02 (Skills API Reference) - High value, unblocks other work
2. TASK-WEB-01 (Terminal Component) - High value, self-contained
3. TASK-AI-01 (Personality System) - Fun, quick win
4. TASK-WEB-02 (LiDAR BEV) - High value, more complex
5. TASK-AI-02 (RAG Core) - High value, but time-intensive

---

## 🔄 Updating This Document

When tasks are completed:
1. Mark task with ✅ status
2. Add completion date
3. Link to PR/commit
4. Note any deviations from spec
5. Add lessons learned

When adding new tasks:
1. Use next sequential task ID
2. Include all sections (Context, Requirements, Deliverables)
3. Verify no ROS dependencies
4. Add to priority matrix

---

*This document is a living guide. Update as new agent-friendly tasks are identified or requirements change.*
