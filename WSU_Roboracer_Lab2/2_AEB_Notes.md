---
sort: 2
---

# AEB NOTES

To set up your safety package within the driver stack container and prevent the roboracer from colliding with objects in front of it, follow these steps:

### **1️⃣ Create the Safety Package**
   
Inside the driver stack container, navigate to your ROS 2 workspace  ~/f1tenth_ws/src and create a new package:

```bash 
cd ~/f1tenth_ws/src
ros2 pkg create safety_package --build-type ament_python --dependencies rclpy sensor_msgs std_msgs
```

```note
Dependencies
- rcply (ROS Client Library for Python)
- sensor_msgs (Standard Messages for Sesnsors)
```

### **2️⃣ Modify package.xml**

Ensure package.xml includes dependencies like rclpy, sensor_msgs, and std_msgs. Open package.xml and add:

```xml
    <depend>rclpy</depend>
    <depend>sensor_msgs</depend>
    <depend>std_msgs</depend>
```

If you're using C++ instead of Python, also ensure you have:

```xml
    <depend>rclcpp</depend>
    <depend>tf2_ros</depend>
    <depend>geometry_msgs</depend>
```

### **3️⃣ Modify CMakeLists.txt (If Using C++)**
If you're using C++, modify CMakeLists.txt to include:

```bash
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
```

Ensure the add_executable or ament_target_dependencies includes the necessary dependencies.

### **4️⃣ Install Dependencies Using rosdep**
Run the following to install missing dependencies:

```bash
cd ~/f1tenth_ws
rosdep install --from-paths src --ignore-src -r -y
```

### **5️⃣ Implement the Safety Node**
You’ll create a safety node that listens to LiDAR data (/scan) and publishes a safety brake command if an object is too close.

Example: Python Safety Node (safety_node.py)
Create a file inside safety_package/safety_node.py:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # TODO: create ROS subscribers and publishers.

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = 0.

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        
        # TODO: publish command to brake
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **6️⃣ Make It Executable**
Modify setup.py inside safety_package:

```python
entry_points={
    'console_scripts': [
        'safety_node = safety_package.safety_node:main',
    ],
},
```

### **7️⃣ Build & Run**
Run the following:

```bash
cd ~/f1tenth_ws
colcon build --packages-select safety_package
source install/setup.bash
ros2 run safety_package safety_node
```

This will listen to LiDAR (/scan) and publish a safety stop signal (/safety_brake) when an obstacle is too close.


