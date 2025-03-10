---
sort: 3
---

# AEB Drive Example


---

## Full Code

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class DriveForwardNode(Node):
    """
    A ROS 2 node that continuously commands the vehicle to drive forward.
    """

    def __init__(self):
        super().__init__('drive_forward_node')

        # ✅ Publisher for drive commands
        self.drive_publisher_ = self.create_publisher(
            AckermannDriveStamped, "/drive", 10)

        # ✅ Timer to continuously send drive commands (at 10Hz)
        self.drive_timer = self.create_timer(0.1, self.drive_callback)

        self.get_logger().info("✅ Drive Forward Node Started! Sending drive commands...")

    def drive_callback(self):
        """Publishes a drive command to move forward."""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 1.0  # Move forward at 1.0 m/s
        drive_msg.drive.steering_angle = 0.0  # Keep wheels straight

        self.drive_publisher_.publish(drive_msg)
        self.get_logger().info("🚗 Driving Forward: speed 1.0 m/s")


def main(args=None):
    rclpy.init(args=args)
    drive_node = DriveForwardNode()
    rclpy.spin(drive_node)
    drive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


```