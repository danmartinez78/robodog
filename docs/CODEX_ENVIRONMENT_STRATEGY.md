# Codex Development Strategy for Ubuntu 24.04

## Background
The current developer workflow relies on a VS Code devcontainer built on Ubuntu 22.04 with ROS 2 Humble, Navigation2, CycloneDDS, and the DIMOS framework preinstalled to support the Unitree Go2 mission stack.【F:README.md†L48-L66】 Codex workspaces, however, currently default to Ubuntu 24.04 images where ROS 2 Humble is unsupported, which prevents agents from reproducing the full container setup. This document maps out what Codex agents *can* do inside the 24.04 environment and where human-in-the-loop validation inside the 22.04/ROS container remains necessary.

## Project Snapshot
- The `feature/dimos-integration` branch delivered the DIMOS-based mission agent and bringup packages along with extensive documentation, enabling natural-language mission execution by reusing DIMOS’s 40+ Unitree skills and infrastructure.【F:docs/MERGE_SUMMARY.md†L3-L115】
- The recent `feature/agent-refactor` work split the mission agent into a pure-Python `MissionExecutor` (business logic) and a thin ROS node wrapper, backed by 14 unit tests that run without ROS dependencies.【F:DEVLOG.md†L7-L45】【F:src/shadowhound_mission_agent/test/test_mission_executor.py†L1-L200】
- The MVP plan now emphasizes Nav2-based autonomy with add-on vision skills (snapshot, detect, describe) and future lidar integrations to support missions such as “explore the kitchen and report back.”【F:docs/mvp_plan_pivot.md†L1-L111】

## Dependency Profile by Component
| Component | Primary Language | External Requirements | 24.04 Feasibility |
| --- | --- | --- | --- |
| `MissionExecutor` core logic | Python | DIMOS (imported lazily) | ✅ Unit tests already use mocking to run without DIMOS/ROS, so Codex can extend or refactor logic safely.【F:DEVLOG.md†L14-L29】【F:src/shadowhound_mission_agent/test/test_mission_executor.py†L59-L158】 |
| Web interface (`web_interface.py`) | Python/FastAPI | FastAPI, uvicorn | ✅ Fully self-contained; can be started and tested in 24.04 with pip-installed deps.【F:src/shadowhound_mission_agent/shadowhound_mission_agent/web_interface.py†L1-L200】 |
| Vision skills (`vision.py`) | Python, NumPy/Pillow | Optional DIMOS Qwen VLM; snapshot skill works offline | ✅ Snapshot pipeline is ROS-free; Codex can add image-processing utilities and tests using mock images.【F:src/shadowhound_skills/shadowhound_skills/vision.py†L21-L134】 |
| DIMOS VLM calls | Python | Alibaba Qwen API key | ⚠️ Network/API access depends on secrets; Codex can write wrappers/tests with mocks but cannot run live calls. |
| Launch packages (`shadowhound_bringup`, ROS launch files) | Python/ROS | ROS 2 Humble, colcon | ❌ Need ROS Humble; must be validated in 22.04 devcontainer. |
| Nav2 + robot skills validation | ROS C++/Python | Real robot or ROS simulation | ❌ Requires ROS stack and hardware/sim; outside Codex 24.04 scope. |

## High-Value Tasks Suitable for Codex Agents
1. **Finish high-priority unit tests.** TODOs call out four skipped tests covering mission pause/resume, error recovery, and telemetry.【F:TODO.md†L15-L18】 Codex can flesh out these tests by mocking DIMOS interfaces in 24.04, improving coverage without ROS.
2. **Implement mission history and replay.** Recording mission logs and providing replay/export features is a pure-Python problem aligned with existing TODOs, ideal for Codex to prototype alongside the web interface.【F:TODO.md†L20-L23】
3. **Enhance vision utilities.** With snapshot skills already independent of DIMOS, Codex can add panorama stitching, file management, or NumPy-based analysis that run locally, and write unit tests against stored mock frames.【F:src/shadowhound_skills/shadowhound_skills/vision.py†L47-L134】【F:docs/VISION_INTEGRATION_DESIGN.md†L9-L122】
4. **Extend the FastAPI dashboard.** Codex can enrich the mission control UI (e.g., mission queue management, telemetry graphs) because the server is pure Python and ships with mock-image upload hooks for testing.【F:src/shadowhound_mission_agent/shadowhound_mission_agent/web_interface.py†L38-L200】
5. **Documentation and tooling updates.** Medium-priority tasks include README and skill reference refreshes, which Codex can draft directly in Markdown without ROS access.【F:TODO.md†L51-L66】

## Work Requiring the 22.04/ROS Container or Hardware
- ROS launch flows, Nav2 behavior trees, and hardware-level skill validation remain tied to ROS Humble and real Go2 access.【F:TODO.md†L25-L41】【F:docs/mvp_plan_pivot.md†L36-L64】
- DIMOS perception stacks that depend on CUDA or rosdep resolution (documented as non-blocking issues during the integration merge) cannot be exercised inside Codex’s 24.04 images.【F:docs/MERGE_SUMMARY.md†L55-L138】

## Recommended Workflow Split
1. **Codex (Ubuntu 24.04) Responsibilities**
   - Maintain pure-Python packages (`shadowhound_mission_agent`, `shadowhound_skills` vision utilities) and associated unit tests.
   - Prototype FastAPI/web dashboard features and mission logging/reporting flows.
   - Prepare documentation updates, design notes, and config templates.
   - Provide mocked DIMOS bindings or adapters so code paths stay testable without ROS.
2. **Human/22.04 Container Responsibilities**
   - Run `colcon build/test`, ROS launch files, and Nav2 mission dry-runs.
   - Integrate and validate DIMOS skill execution, WebRTC vs Nav2 mode switching, and any hardware-dependent logic.
   - Manage rosdep, CUDA, and other ROS/Humble-only dependencies.

## Tooling Suggestions for Codex Environments
- Use `uv`/`pip` to install FastAPI, uvicorn, numpy, pillow, pytest, and any mock dependencies when running tests in 24.04.
- Rely on the existing unit-test suite (`pytest src/shadowhound_mission_agent/test`) as a regression harness after each change; add new suites for vision utilities and mission history as features land.【F:src/shadowhound_mission_agent/test/test_mission_executor.py†L1-L200】
- Maintain compatibility by avoiding ROS imports in pure-Python modules unless they are guarded by `try/except` like the current DIMOS vision integration does.【F:src/shadowhound_skills/shadowhound_skills/vision.py†L21-L64】

## Next Steps
1. Draft the missing mission executor tests and mission history scaffolding via Codex to close the high-priority TODO items.【F:TODO.md†L15-L23】
2. Add a lightweight mock DIMOS client interface so the same mission logic can be exercised in both Codex (24.04) and the ROS container without code changes.【F:DEVLOG.md†L14-L29】
3. Queue follow-up validation sessions inside the 22.04 container to run ROS launch files and Nav2 missions once Codex-delivered features land.【F:docs/mvp_plan_pivot.md†L36-L64】

