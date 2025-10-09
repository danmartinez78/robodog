#!/usr/bin/env python3
"""
Trigger costmap publication by sending a small motion command.
This helps ensure costmaps are published before DIMOS initializes.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys


class CostmapTrigger(Node):
    def __init__(self):
        super().__init__("costmap_trigger")
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Give publisher time to connect
        time.sleep(0.5)
        
    def trigger(self):
        """Send a tiny rotation command to trigger costmap updates."""
        print("📡 Triggering costmap publication...")
        print("   Sending small rotation command to wake up Nav2...")
        
        # Very small angular velocity for 0.5 seconds
        twist = Twist()
        twist.angular.z = 0.1  # 0.1 rad/s rotation
        
        # Send command for a brief moment
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        print("✓ Costmap trigger complete")
        print("   (Robot rotated ~3 degrees, Nav2 should now publish costmaps)")


def main():
    rclpy.init()
    
    try:
        node = CostmapTrigger()
        node.trigger()
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n❌ Error: {e}")
        return 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
