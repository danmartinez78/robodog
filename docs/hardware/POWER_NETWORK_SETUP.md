# ShadowHound Power & Network Setup

## Overview
This document outlines a practical power and network configuration for running the **AGX Thor**, **Intel RealSense D555**, and **GL.iNet GL-SFT1200 (Opal)** router using a single 140 W dual USB-C power bank. The goal is to achieve a fully wired, deterministic low-latency setup for indoor robotics and Holoscan development.

---

## Network Topology

```
            +-----------------------+
            |  140 W Power Bank     |
            |  (2 × USB-C PD, 1 × A)|
            +----------+------------+
                       | 
        ┌──────────────┼───────────────────────────────┐
        |              |                               |
        |              |                               |
  USB-C PD → Thor  USB-C PD → PoE Injector → D555     USB-A → Router
        |                |             |                 |
        |                |             |                 |
        |                |             |                 |
        |           Ethernet (Cat6)    |                 |
        |<────────────── LAN (GigE) ────────────────────>|
```

### Device IP Assignments (Static Example)
| Device | Interface | IP Address |
|---------|------------|-------------|
| GL-SFT1200 Router | LAN bridge | 192.168.10.1 |
| AGX Thor | eth0 | 192.168.10.3 |
| GO2 (onboard port) | eth0 | 192.168.123.161 |
| RealSense D555 | eth0 (PoE) | 192.168.10.4 |

> Note: If an external compute dock (e.g., Xavier Backpack / EDU version) is present, its default IP is 192.168.123.18. For the base robot without this dock, the onboard computer interface uses 192.168.123.161.

Router should run in **Access Point (Bridge) mode**, merging WAN and LAN ports into one subnet.

---

## Power Distribution

### Power Sources
| Port | Output | Device | Notes |
|------|---------|---------|-------|
| USB-C PD #1 | 20 V / up to 7 A | AGX Thor | Primary compute node |
| USB-C PD #2 | 20 V / 2–3 A | PD→PoE+ Injector | Powers D555 via Ethernet |
| USB-A | 5 V / 2 A | GL-SFT1200 Router | Low current draw (<5 W) |

### Recommended PoE Injector
**UCTRONICS USB-C PD to PoE+ Injector (U6116)**  
- PD input: 20 V (negotiated automatically)  
- Output: IEEE 802.3at 48 V PoE+ up to 25 W  
- True Gigabit data passthrough  
- Compact and field-tested for cameras and embedded devices.

**Alternative:** iCreatin USB-C PD to 48 V PoE+ Injector (30 W)

### Wiring Notes
- Use **Cat6 or better** cables (D555 datasheet specifies GigE 1000BASE-T with jumbo frame support).
- PoE injector should sit inline between router and D555.
- The D555's USB port is for **debug and factory use only**; normal operation runs via PoE Ethernet.

---

## Verification Steps
1. Connect all Ethernet links and confirm **link LEDs** are active.
2. Log into the GL-SFT1200 (`192.168.8.1`) and set **Access Point Mode**.
3. Assign static IPs to Thor, GO2, and D555.
4. Verify LAN connectivity:
   ```bash
   ping 192.168.123.161   # GO2 onboard port
   ping 192.168.10.4      # D555
   ```
5. Test throughput:
   ```bash
   iperf3 -c 192.168.10.3 -t 30
   ```
   Expect >900 Mbps and <1 ms latency.
6. Launch RealSense SDK 2.0 with DDS backend to confirm streaming over Ethernet.

---

## Key Advantages
- **Fully wired, deterministic network** (no Wi-Fi jitter)
- **Shared power domain** with proper voltage isolation
- **Native PoE+ powering** of D555 as per datasheet spec
- **Simple field setup:** one battery, one router, one injector

---

## Future Expansion
- Replace GL-SFT1200 with a multi-gig router (e.g., GL-BE3600 Slate 7) for higher bandwidth.
- Add a precision time protocol (PTP) daemon on Thor to synchronize timestamps for Holoscan and D555.
- Integrate power telemetry to monitor draw per device.

---

*This configuration has been validated against the D555 Datasheet v1.1 (Power over Ethernet 802.3at, Gigabit, DDS support) and Unitree GO2 documentation indicating onboard IP address 192.168.123.161.*
