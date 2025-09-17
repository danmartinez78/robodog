# AGENTS.md — Copilot Coding Agent Guide for ShadowHound
_Last updated: 2025-09-16

This guide helps you (and GitHub Copilot/Copilot Chat) work effectively in the **ShadowHound** repo (GO2 + Orin Nano + Thor/PC). It standardizes prompts, conventions, and task recipes so the agent can scaffold code, tests, and docs with minimal back-and-forth.

---

## 1) Project Snapshot (for Copilot context)
- **Robot**: Unitree GO2 Pro (jailbroken) with Jetson **Orin Nano 8GB** on-board.
- **Brains**: Jetson **AGX Thor** and/or **GPU PC** for LLM/VLM/VLA.
- **Base SDK**: Fork of `abizovnuralem/go2_ros2_sdk` (ROS 2).
- **References**: `ros2_jetbot_tools`, `ros2_jetbot_voice`.
- **Networking**: Orin↔GO2 wired (static /24), Wi‑Fi for dev laptop; topics visible over Wi‑Fi.
- **Reasoning I/O**: **gRPC/HTTP** to model server (avoid large blobs on ROS topics).

> Copilot: _Always read_ `docs/project_context.md` before making assumptions. Prefer ROS 2 **Iron/Humble** + **CycloneDDS** unless configured otherwise.

---

## 2) Conventions & Repo Layout
- Language: **Python** for glue/ROS nodes; **C++** only for performance-critical paths.
- Build: **colcon** with `ament` (keep packages minimal and well-named).
- ROS 2 packages (planned):
  - `shadowhound_request_builder` (py): builds compact reasoning requests.
  - `shadowhound_executor` (py): turns high-level plans into waypoints/skills.
  - `shadowhound_reasoner_client` (py): gRPC client to Thor/PC.
  - `shadowhound_utils` (py): shared msgs, QoS utils, compression helpers.
- Launch files live in `pkg_name/launch/*.launch.py`.
- Configs in `pkg_name/config/` (YAML, DDS XML, netplan snippets, gRPC endpoints).
- Tests in `pkg_name/test/` using `pytest` (python) or `ament_add_gtest` (c++).

**Coding standards**
- Python: `ruff` + `black`, type hints (`mypy` optional).
- C++: `ament_uncrustify` + `-Wall -Wextra -Werror`.
- ROS 2: prefer **composition** (components) where possible; QoS explicit.

---

## 3) Copilot Prompt Patterns (copy/paste)
Use these in **Copilot Chat** from the repo root or inside the target package.

### A. Create a ROS 2 Python package
```
Create a ROS 2 Python package named `shadowhound_request_builder` with minimal structure.
- Nodes: request_builder_node.py (rclpy)
- Declared entrypoint console_scripts: request_builder
- Dependencies: rclpy, sensor_msgs, nav_msgs, std_msgs
- Include pyproject.toml, setup.cfg (black/ruff), package.xml, setup.py
- Add a launch file request_builder.launch.py
- Add basic README with run commands
```
Follow up:
```
Write request_builder_node.py that subscribes to:
- /detected_objects (std_msgs/String for now, JSON payload)
- /local_map_pose (geometry_msgs/PoseStamped)
Publishes:
- /reasoning/request (std_msgs/String JSON: scene summary + constraints)
Add unit tests for JSON schema building.
```

### B. gRPC client/server stubs
```
Generate a minimal gRPC `.proto` for a Reasoner service:
service Reasoner { rpc Plan(ReasoningRequest) returns (HighLevelPlan); }
Messages include: robot_id, instruction, scene (objects, pose), thumbnails (bytes or URIs), constraints.
Produce:
- proto/plan.proto
- Python server (thor_pc/reasoner_server.py) using grpcio
- Python client node (shadowhound_reasoner_client/client_node.py) calling the server
- Launch: reasoner_client.launch.py with server address param
- Add tests with a fake server returning a fixed plan
```

### C. Executor scaffolding
```
Create `shadowhound_executor` with a node that subscribes to `/reasoning/plan` (std_msgs/String JSON).
Convert steps of type "goto" into geometry_msgs/PoseStamped on `/nav/goal`.
Add a simple rate limiter and a watchdog timeout.
```

### D. Networking helpers
```
Add a pkg `shadowhound_utils` with:
- dds_qos.py: utilities to build QoS profiles
- net.py: helpers to detect wired vs wifi IPs, set ROS_DOMAIN_ID from env
- image.py: thumbnail compression (JPEG, quality=70), base64 helpers
Provide unit tests.
```

### E. Voice hooks (Phase 1)
```
Add a launch example integrating ros2_jetbot_voice as a submodule or external node.
Expose topics: /speech/transcript (String), /speech/say (String).
Bridge /speech/transcript -> request_builder instruction input.
```

---

## 4) AI-Generated PR Checklist (paste into PRs)
- [ ] Code compiles (`colcon build`) and tests pass.
- [ ] Launch files start without missing params.
- [ ] Topics/services named and documented.
- [ ] QoS profiles explicit for all pubs/subs.
- [ ] Node has clean shutdown and log levels configurable.
- [ ] No large binaries or secrets committed.
- [ ] README updated with run commands and env vars.
- [ ] Added/updated tests and sample bags if relevant.

---

## 5) Run Commands (quick reference)
```bash
# build
colcon build --symlink-install
source install/setup.bash

# run request builder
ros2 run shadowhound_request_builder request_builder

# run reasoner client (point at Thor/PC)
ros2 launch shadowhound_reasoner_client reasoner_client.launch.py server_host:=thor.local server_port:=50051

# run executor
ros2 run shadowhound_executor executor
```

> Set `ROS_DOMAIN_ID=7` and ensure Wi‑Fi allows discovery, or configure CycloneDDS/Discovery Server.

---

## 6) Env Vars & Config
- `ROS_DOMAIN_ID=7`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `REASONER_HOST=thor.local` (or IP)
- `REASONER_PORT=50051`
- `SH_THUMBNAIL_JPEG_QUALITY=70`
- DDS XML (optional) to pin or widen interfaces in `docs/networking/dds/*.xml`.

---

## 7) Guardrails for Copilot
- **Do not** stream camera frames over ROS to Thor/PC; send **thumbnails/URIs** via gRPC.
- **Do not** assume multicast works on all Wi‑Fi; provide a Discovery-Server fallback.
- **Prefer** small, composable nodes over monoliths.
- **Log** latency and payload sizes for all cross-machine calls.
- **Keep** secrets and device-specific IPs out of code; use params/env.

---

## 8) Troubleshooting
- **No topics visible from laptop**: check ROS_DOMAIN_ID, Wi‑Fi multicast, firewall; or use Discovery Server.
- **gRPC timeouts**: verify server host/port, MTU, payload size.
- **High latency**: pin heavy topics to Ethernet; compress thumbnails.
- **Build fails on Orin**: match ROS 2 distro with Dev PC; ensure JetPack CUDA/TensorRT present.

---

## 9) Example Schemas

**ReasoningRequest (JSON)**
```json
{"robot_id":"go2-pro-01","instruction":"Inspect bay A",
 "scene":{"objects":[{"id":"rack01","class":"rack"}],
           "map_pose":{"x":1.2,"y":-3.4,"yaw":0.8}},
 "thumbnails":["<base64-jpeg>"],
 "constraints":{"max_time_s":20,"energy_budget_pct":15}}
```

**HighLevelPlan (JSON)**
```json
{"plan_id":"abc123","steps":[
  {"type":"goto","target":{"x":2.1,"y":-3.0,"yaw":0.0}},
  {"type":"perceive","target":"rack01","mode":"photo+detect"},
  {"type":"report","channel":"ops_log","summary":"Rack A has 3/4 slots filled"}
],
"safety_notes":["avoid no-go at y<-4.0"],
"fallback":{"if_timeout_s":10,"action":"return_to_base"}}
```

---

## 10) Prompting Tips
- State **goal + constraints + files to touch**.
- Ask for **diff-sized changes** and **tests**.
- Provide **example inputs/outputs**.
- End with **acceptance criteria** and **run commands**.

