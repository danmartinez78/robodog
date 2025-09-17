---
name: Voice I/O — Reference Integration
about: Plan voice capabilities using ros2_jetbot_voice as reference
title: "Voice I/O: plan integration"
labels: ["voice", "phase1"]
assignees: []
---

## Goals
- Survey `ros2_jetbot_voice` patterns for ASR/TTS on Jetson
- Decide where ASR/TTS runs (Orin vs Thor/PC)
- Define ROS 2 topics/services for speech

## Checklist
- [ ] Add `ros2_jetbot_voice` to references
- [ ] Prototype mic device detection on Orin
- [ ] Choose ASR (e.g., Whisper) and TTS (e.g., Coqui) backends
- [ ] Draft launch file that exposes `/speech/transcript` and `/speech/say`
- [ ] Record a short demo

## Notes
Paste links and benchmarks.
