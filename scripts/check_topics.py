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
            "/go2_states": "Robot state data (30Hz)",
            "/camera/image_raw": "Camera feed (15Hz)",
            "/imu": "IMU data",
            "/odom": "Odometry",
            "/local_costmap/costmap": "Local costmap (Nav2)",
            "/cmd_vel_out": "Velocity commands",
        }

        print("🎯 Critical topic status:")
        all_found = True
        for topic, description in critical_topics.items():
            found = any(name == topic for name, _ in topic_names_and_types)
            status = "✅" if found else "❌"
            print(f"  {status} {topic:<30} {description}")
            if not found:
                all_found = False

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

        if all_found:
            print("✅ ALL CRITICAL TOPICS FOUND - Ready for mission agent!")
        else:
            print("⚠️  SOME CRITICAL TOPICS MISSING")
            print("   Make sure the robot driver is fully launched")

        print("=" * 70)


def main():
    rclpy.init()

    try:
        node = TopicDiagnostics()
        # Don't spin - we just want to check topics once
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n❌ Error: {e}")
        return 1
    finally:
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
