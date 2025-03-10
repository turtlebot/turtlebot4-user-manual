---
sort: 4
---

# AEB Drive Example

### **Code Explanation: `DriveForwardNode`**

This Python script is a **ROS 2 node** that continuously commands a vehicle to drive forward using the **AckermannDriveStamped** message.

---

### **1️⃣ Shebang & Imports**

```
python
#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from ackermann_msgs.msg import AckermannDriveStamped
```
- `#!/usr/bin/env python3` → Specifies that this script should be executed using Python 3.
- `import rclpy` → Imports the ROS 2 Python client library.
- `from rclpy.node import Node` → Imports the `Node` class, which is the base class for all ROS 2 nodes.
- `from ackermann_msgs.msg import AckermannDriveStamped` → Imports the `AckermannDriveStamped` message type, which is used to control vehicles with **Ackermann steering (used in cars)**.

---

### **2️⃣ Class Definition: `DriveForwardNode`**

```python
class DriveForwardNode(Node):
```

- This defines a ROS 2 **node** called `DriveForwardNode`.
- It inherits from `Node`, meaning it has ROS 2 functionalities like **publishing, subscribing, and logging**.

---

### **3️⃣ Node Initialization (`__init__`)**

```python
def __init__(self):     super().__init__('drive_forward_node')
```

- Calls the parent `Node` constructor and **names the node** as `"drive_forward_node"`.
- This **registers the node** in the ROS 2 system.

---

### **4️⃣ Creating a Publisher**

```python
self.drive_publisher_ = self.create_publisher(AckermannDriveStamped, "/drive", 10)
```

- Creates a **ROS 2 publisher** that sends messages of type `AckermannDriveStamped` to the `/drive` topic.
- `10` is the **queue size**, meaning up to 10 messages will be buffered if the subscriber is slow.

---

### **5️⃣ Creating a Timer**

```python
self.drive_timer = self.create_timer(0.1, self.drive_callback)
```

- Creates a **ROS 2 timer** that calls `self.drive_callback` **every 0.1 seconds (10Hz)**.
- This ensures that the car keeps receiving **drive commands** continuously.

---

### **6️⃣ Logging Node Startup**

```python
self.get_logger().info("✅ Drive Forward Node Started! Sending drive commands...")
```
- Logs a message indicating that the node has started.

---

### **7️⃣ Drive Command Callback (`drive_callback`)**

```python
def drive_callback(self):     
    """Publishes a drive command to move forward."""     
    drive_msg = AckermannDriveStamped()     
    drive_msg.drive.speed = 1.0  # Move forward at 1.0 m/s     
    drive_msg.drive.steering_angle = 0.0  # Keep wheels straight      
    self.drive_publisher_.publish(drive_msg)     
    self.get_logger().info("🚗 Driving Forward: speed 1.0 m/s")
```

- **Creates** a `drive_msg` of type `AckermannDriveStamped`.
- **Sets speed** to `1.0 m/s` (moves forward).
- **Sets steering angle** to `0.0` (keeps the car straight).
- **Publishes the message** to `/drive`, commanding the car to move forward.
- **Logs the action** for debugging.

---

### **8️⃣ Main Function**

```python
def main(args=None):     
    rclpy.init(args=args)  # Initialize ROS 2     
    drive_node = DriveForwardNode()  # Create node instance     
    rclpy.spin(drive_node)  # Keep node running     
    drive_node.destroy_node()  # Cleanup on shutdown     
    rclpy.shutdown()  # Shutdown ROS 2`
```

- `rclpy.init(args=args)` → Initializes ROS 2.
- `drive_node = DriveForwardNode()` → Instantiates the `DriveForwardNode`.
- `rclpy.spin(drive_node)` → Keeps the node **running indefinitely** (until stopped).
- `drive_node.destroy_node()` → Cleans up the node before shutdown.
- `rclpy.shutdown()` → Shuts down ROS 2.

---

### **9️⃣ Running the Script**

```python
if __name__ == '__main__':     
    main()
```

- Ensures that the `main()` function **only runs** if the script is executed directly (not imported as a module).

---

### **💡 Summary**

|**Component**|**Purpose**|
|---|---|
|`create_publisher()`|Sends drive commands to `/drive` topic|
|`create_timer(0.1, self.drive_callback)`|Calls `drive_callback()` every 0.1s (10Hz)|
|`drive_callback()`|Publishes a forward motion command|
|`rclpy.spin(drive_node)`|Keeps the node running indefinitely|

This node **continuously** commands a car to drive **forward at 1.0 m/s**. 🚗💨
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