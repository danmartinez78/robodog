---
name: Networking — Wired + Wi‑Fi + ROS 2 Discovery
about: Configure dual-interface networking and topic visibility
title: "Networking: Orin↔GO2 wired + Wi‑Fi visualization"
labels: ["networking", "phase0"]
assignees: []
---

## Goals
- Orin↔GO2 over Ethernet on static subnet (e.g., 192.168.50.0/24)
- Orin and laptop on Wi‑Fi for SSH/RViz
- ROS 2 topics visible from laptop

## Checklist
- [ ] Set static IPs on Orin (`eth0`) and GO2
- [ ] Configure Wi‑Fi on Orin (`wlan0`) to lab SSID
- [ ] Set `ROS_DOMAIN_ID=7` on all devices
- [ ] Choose RMW (CycloneDDS or Fast DDS) consistently
- [ ] If multicast blocked, configure Discovery Server or CycloneDDS unicast
- [ ] Confirm `ros2 topic list` and RViz from laptop over Wi‑Fi
- [ ] Save `netplan` files and DDS configs to `docs/networking/`

## Artifacts
- Paste `ip addr`, `route -n`, and DDS XML or server config here.
