#!/usr/bin/env python3
"""
Quick diagnostic tool to check ROS2 topic and action visibility.
Run this before launching the mission agent to verify the robot driver is working.
"""

import rclpy
from rclpy.node import Node
import sys


class TopicDiagnostics(Node):
    def __init__(self):
        super().__init__("topic_diagnostics")
        self.all_critical_found = True
    
    def check_topics(self):
        print("=" * 70)
        print("🔍 ROS2 TOPIC & ACTION DIAGNOSTICS")
        print("=" * 70)
        print()

        # Get all topics
        topic_names_and_types = self.get_topic_names_and_types()

        # Filter for robot-related topics
        robot_topics = [
            (name, types)
            for name, types in topic_names_and_types
            if any(
                keyword in name
                for keyword in [
                    "go2",
                    "camera",
                    "imu",
                    "odom",
                    "costmap",
                    "cmd_vel",
                    "webrtc",
                    "joint",
                    "point_cloud",
                    "scan",
                ]
            )
        ]

        print(f"📡 Found {len(robot_topics)} robot-related topics:")
        if robot_topics:
            for name, types in sorted(robot_topics):
                print(f"  ✓ {name}")
                for msg_type in types:
                    print(f"      └─ {msg_type}")
        else:
            print("  ❌ No robot topics found!")
            print("  ⚠️  Is the robot driver running?")
            print()
            print("  Start with: ros2 launch go2_robot_sdk robot.launch.py")

        print()

        # Check for specific critical topics
        critical_topics = {
            "/go2_states": ("Robot state data (30Hz)", True),
            "/camera/image_raw": ("Camera feed (15Hz)", True),
            "/imu": ("IMU data", True),
            "/odom": ("Odometry", True),
            "/cmd_vel_out": ("Velocity commands", True),
        }

        optional_topics = {
            "/local_costmap/costmap": (
                "Local costmap (Nav2, updates on movement)",
                False,
            ),
            "/global_costmap/costmap": (
                "Global costmap (Nav2, updates on movement)",
                False,
            ),
        }

        print("🎯 Critical topic status:")
        all_critical_found = True
        for topic, (description, required) in critical_topics.items():
            found = any(name == topic for name, _ in topic_names_and_types)
            status = "✅" if found else "❌"
            print(f"  {status} {topic:<30} {description}")
            if not found and required:
                all_critical_found = False
        
        self.all_critical_found = all_critical_found

        print()
        print("📍 Optional topics (may appear on movement):")
        for topic, (description, _) in optional_topics.items():
            found = any(name == topic for name, _ in topic_names_and_types)
            status = "✅" if found else "⚪"
            print(f"  {status} {topic:<30} {description}")

        print()

        # Check action servers
        print("🎮 Checking Nav2 action servers...")
        try:
            import time
            from rclpy.action import ActionClient
            from nav2_msgs.action import Spin

            # Give action servers time to advertise
            time.sleep(0.5)

            spin_client = ActionClient(self, Spin, "spin")
            spin_available = spin_client.wait_for_server(timeout_sec=2.0)
            spin_client.destroy()

            status = "✅" if spin_available else "❌"
            print(f"  {status} /spin action server")

            if not spin_available:
                print("  ⚠️  Nav2 /spin action server not found")
                print("  ℹ️  This is needed for rotation commands")

        except Exception as e:
            print(f"  ❌ Error checking action servers: {e}")

        print()
        print("=" * 70)

        if self.all_critical_found:
            print("✅ ALL CRITICAL TOPICS FOUND - Ready for mission agent!")
        else:
            print("⚠️  SOME CRITICAL TOPICS MISSING")
            print("   Make sure the robot driver is fully launched")

        print("=" * 70)

        return self.all_critical_found


def main():
    rclpy.init()

    all_ok = False
    try:
        node = TopicDiagnostics()
        all_ok = node.check_topics()
        # Don't spin - we just want to check topics once
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n❌ Error: {e}")
        return 1
    finally:
        rclpy.shutdown()

    # Return non-zero exit code if critical topics are missing
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
