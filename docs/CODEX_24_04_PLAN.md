# Codex 24.04 Enablement Plan

## 1. Environment Reality
- The project devcontainer is pinned to **ROS 2 Humble** images (`ros:${ROS_DISTRO}-${VARIANT}`) which are based on Ubuntu 22.04, so Humble tooling is expected in local development. 【F:.devcontainer/Dockerfile†L2-L107】
- Codex only exposes an Ubuntu 24.04 base image. Because Humble packages and binaries target 22.04, we need to focus Codex tasks on components that either avoid ROS entirely or can be exercised with mocks/stubs.

## 2. Branch Highlights & Existing Work
- **feature/dimos-integration** delivered the full DIMOS bridge, ROS bringup package, and usage docs, reusing DIMOS skills/agents and only adding a light ShadowHound layer. 【F:docs/INTEGRATION_STATUS.md†L9-L175】
- **feature/agent-refactor** separated ROS plumbing from business logic by introducing the pure-Python `MissionExecutor`, added fast unit tests, and simplified the ROS node wrapper. 【F:DEVLOG.md†L7-L45】
- **feature/vlm-integration** created the `shadowhound_skills` package with four vision skills, a full test suite, and clear optional dependency boundaries around DIMOS vision APIs. 【F:docs/VLM_INTEGRATION_SUMMARY.md†L7-L200】
- Web dashboard work lives alongside the mission agent and runs inside the ROS node process but is implemented as a standalone FastAPI module that can be launched without ROS. 【F:docs/WEB_INTEGRATION_SUMMARY.md†L11-L170】

## 3. Components Safe for Codex Development
| Area | Why It Works on 24.04 | Suggested Activities |
|------|-----------------------|----------------------|
| **MissionExecutor (shadowhound_mission_agent)** | Pure Python module that deliberately avoids ROS and can be instantiated directly; ROS dependencies are mocked and guarded by import checks. 【F:src/shadowhound_mission_agent/shadowhound_mission_agent/mission_executor.py†L2-L135】 | Extend unit tests, design mission history/telemetry hooks, add configuration validation, or simulate mission flows with mock DIMOS objects. |
| **MissionExecutor Architecture Docs** | Documentation clarifies the separation of ROS vs business logic, providing guidance for non-ROS usage. 【F:src/shadowhound_mission_agent/AGENT_ARCHITECTURE.md†L1-L152】 | Keep docs in sync with new pure-Python capabilities, draft tutorials for notebook/script usage. |
| **Vision Skills (shadowhound_skills)** | Snapshot skill works with Pillow/numpy only; DIMOS-dependent VLM skills are optional and fail gracefully if unavailable. Tests skip API-key-required flows. 【F:src/shadowhound_skills/shadowhound_skills/vision.py†L21-L134】【F:src/shadowhound_skills/test/test_vision.py†L51-L189】 | Enhance snapshot tooling, add offline image processing utilities, refactor for improved error reporting, or broaden tests around numpy/PIL handling. |
| **Vision Skills Documentation/Test Harness** | Summary doc and test runner already demonstrate standalone execution paths. 【F:docs/VLM_INTEGRATION_SUMMARY.md†L61-L200】 | Automate mock responses, enrich documentation with 24.04-specific setup notes, or add synthetic datasets. |
| **Web Interface Module** | FastAPI/uvicorn stack requires only Python; mission callbacks can be mocked. 【F:docs/WEB_INTEGRATION_SUMMARY.md†L34-L170】 | Iterate on UI/REST features, add API contract tests, implement authentication stubs, or create non-ROS demos. |

## 4. Recommended Codex Workstreams
1. **Finish the Pending Unit Tests** – The refactor left four skipped MissionExecutor tests (pause/resume, recovery, telemetry) that can be implemented with mocks and pytest. 【F:TODO.md†L15-L18】
2. **Design Mission History & Replay** – Implement logging/persistence utilities around `MissionExecutor` to satisfy the mission history task without ROS dependencies. 【F:TODO.md†L20-L23】
3. **Document & Script Notebook Usage** – Expand the architecture docs or add tutorials showing how to run missions via scripts/notebooks, leveraging the pure-Python executor. 【F:src/shadowhound_mission_agent/AGENT_ARCHITECTURE.md†L116-L145】
4. **Enhance Vision Snapshot Pipeline** – Improve metadata, compression, and error handling for the Snapshot skill and extend its tests; all can run with numpy/PIL only. 【F:src/shadowhound_skills/shadowhound_skills/vision.py†L85-L134】
5. **Build Web Interface Regression Tests** – Write pytest/FastAPI client tests for REST/WS endpoints using mocked mission callbacks to keep the dashboard stable. 【F:docs/WEB_INTEGRATION_SUMMARY.md†L68-L170】
6. **Draft CI-friendly Test Runner** – Assemble a Python-only test entrypoint (pytest + flake8) that Codex can execute on 24.04 containers, paving the way for future CI. 【F:docs/VLM_INTEGRATION_SUMMARY.md†L23-L78】
7. **Plan Vision-to-Agent Integration** – While full camera wiring needs ROS, Codex can prototype adapters/interfaces that convert numpy frames to skill calls, preparing for later ROS hookup. 【F:docs/VLM_INTEGRATION_SUMMARY.md†L166-L200】

## 5. Work to Defer to ROS Humble / Hardware
- Hardware skill validation, ROS bringup, and mission agent launches require Humble and real robot access from the DIMOS integration. 【F:docs/INTEGRATION_STATUS.md†L31-L115】
- ROS-specific testing (topics, launch files, DDS settings) remains outside Codex scope and should stay in the 22.04 devcontainer.

By focusing Codex runs on the pure-Python layers, we can continue delivering value—tests, docs, utilities, and mock integrations—without waiting on Humble support in Ubuntu 24.04.
