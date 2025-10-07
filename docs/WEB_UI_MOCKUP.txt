┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                        🐕 SHADOWHOUND DASHBOARD                             ┃
┃                    AUTONOMOUS ROBOT MISSION CONTROL                         ┃
┃                            [CONNECTED]                                      ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

┌─────────────────────────────────────┬─────────────────────────────────────┐
│  📹 CAMERA FEED                     │  ⚡ PERFORMANCE METRICS             │
│                                     │                                     │
│  ┌───────────────────────────────┐  │  ┌───────────┬────────────────────┐│
│  │                               │  │  │ AGENT     │ 1.23s       🟢    ││
│  │    [Camera Feed Image]        │  │  ├───────────┼────────────────────┤│
│  │                               │  │  │ OVERHEAD  │ 0.045s      🟢    ││
│  │                               │  │  ├───────────┼────────────────────┤│
│  │                               │  │  │ TOTAL     │ 1.28s       🟢    ││
│  │                               │  │  ├───────────┼────────────────────┤│
│  │                               │  │  │ COMMANDS  │ 42                ││
│  └───────────────────────────────┘  │  └───────────┴────────────────────┘│
│                                     │                                     │
│                                     │  Averages (Last 50 Commands)        │
│                                     │  ┌───────────┬────────────────────┐│
│                                     │  │ AVG AGENT │ 1.45s             ││
│                                     │  ├───────────┼────────────────────┤│
│                                     │  │ AVG TOTAL │ 1.52s             ││
│                                     │  └───────────┴────────────────────┘│
└─────────────────────────────────────┴─────────────────────────────────────┘

┌─────────────────────────────────────┬─────────────────────────────────────┐
│  📊 DIAGNOSTICS                     │  💻 TERMINAL                        │
│                                     │                                     │
│  ROBOT MODE: STANDING               │  [14:32:10] > take one step forward│
│  TOPICS: 12    ACTIONS: 3           │  [14:32:12] ✅ Robot moved forward │
│  LAST UPDATE: 14:32:15              │  [14:32:12] ⏱️  TIMING: Agent     │
│                                     │              1.23s | Overhead      │
│  Available Topics:                  │              0.045s | Total 1.28s  │
│  /cmd_vel  /odom  /camera/image_raw │  [14:32:15] > rotate 90 degrees    │
│  /nav2/goal  /status                │  [14:32:17] ✅ Robot rotated 90°  │
│                                     │  [14:32:17] ⏱️  TIMING: Agent     │
│                                     │              1.15s | Overhead      │
│                                     │              0.038s | Total 1.19s  │
│                                     │  [14:32:20] > stand up             │
└─────────────────────────────────────┴─────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│  📡 MISSION STATUS                                                          │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │ ✅ Robot rotated 90 degrees successfully                              │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│  🎮 COMMAND CENTER                                                          │
│                                                                             │
│  Custom Command                                                             │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │ Enter mission command (e.g., 'stand up and wave hello')              │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│  [▶️ EXECUTE]                                                              │
│                                                                             │
│  Quick Commands                                                             │
│  [🧍 STAND] [🪑 SIT] [👋 WAVE] [💃 DANCE] [🤸 STRETCH] [⚖️ BALANCE]        │
└─────────────────────────────────────────────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════════════
                         PERFORMANCE INSIGHTS
═══════════════════════════════════════════════════════════════════════════════

COLOR CODING:
  🟢 Green  = Good (Agent <2s, Total <3s)
  🟡 Yellow = Warning (Agent 2-4s, Total 3-5s)  ← Consider optimization
  🔴 Red    = Slow (Agent >4s, Total >5s)       ← Needs optimization!

TIMING BREAKDOWN:
  Agent Duration     = Time in DIMOS agent (LLM + skill execution)
  Overhead Duration  = ROS messaging + Python execution time
  Total Duration     = End-to-end mission execution time

EXAMPLE OUTPUTS IN TERMINAL:

After sending "take one step forward":
┌────────────────────────────────────────────────────────────────────┐
│ [14:32:10] > take one step forward                                │
│ [14:32:12] ✅ Robot moved forward successfully                    │
│ [14:32:12] ⏱️  TIMING: Agent 1.23s | Overhead 0.045s | Total 1.28s│
└────────────────────────────────────────────────────────────────────┘

After sending "go to the kitchen":
┌────────────────────────────────────────────────────────────────────┐
│ [14:35:20] > go to the kitchen                                    │
│ [14:35:24] ✅ Navigating to kitchen waypoint                      │
│ [14:35:24] ⏱️  TIMING: Agent 3.45s | Overhead 0.089s | Total 3.54s│
└────────────────────────────────────────────────────────────────────┘

After sending "describe what you see" (with VLM):
┌────────────────────────────────────────────────────────────────────┐
│ [14:40:10] > describe what you see                               │
│ [14:40:15] ✅ I see a hallway with white walls...                │
│ [14:40:15] ⏱️  TIMING: Agent 4.82s | Overhead 0.103s | Total 4.92s│
└────────────────────────────────────────────────────────────────────┘
              ↑
              VLM queries add 1-3s to agent time!


═══════════════════════════════════════════════════════════════════════════════
                         WHAT YOU'LL SEE
═══════════════════════════════════════════════════════════════════════════════

1. IMMEDIATE FEEDBACK in terminal:
   - Command echoed with timestamp
   - Success/failure message
   - Timing breakdown (⏱️ emoji makes it easy to spot)

2. REAL-TIME METRICS in performance panel:
   - Latest timing values update every second
   - Color-coded indicators for quick assessment
   - Running averages smooth out outliers

3. TREND ANALYSIS:
   - Watch averages stabilize over multiple commands
   - Identify if performance degrades over time
   - Compare simple vs complex commands

4. BOTTLENECK IDENTIFICATION:
   - If Agent ≈ Total → Cloud API bottleneck
   - If Overhead > 10% → ROS/Python bottleneck
   - If Agent varies wildly → DIMOS skill bottleneck


═══════════════════════════════════════════════════════════════════════════════
                         NEXT STEPS
═══════════════════════════════════════════════════════════════════════════════

1. Start the system:
   ros2 launch shadowhound_bringup shadowhound_bringup.launch.py

2. Open browser to http://localhost:8080

3. Send test commands and watch metrics update

4. Collect baseline data:
   - Simple commands: "stand", "sit", "wave"
   - Navigation: "go forward", "rotate 90"
   - Complex: "patrol hallway"

5. Analyze results:
   - What's the average agent duration?
   - Is cloud API the bottleneck? (likely yes)
   - Should we optimize before adding VLM? (yes!)

6. Optimize based on findings:
   - If agent >2s → Try gpt-3.5-turbo (faster, cheaper)
   - If overhead high → Profile ROS callbacks
   - Add progress indicators for better UX

7. Then integrate VLM with optimized baseline! 🚀
