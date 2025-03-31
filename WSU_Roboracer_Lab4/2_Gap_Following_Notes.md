---
sort: 2
---

# Wall Following NOTES

To set up your wall following package within the driver stack container follow these steps:

### **1️⃣ Create the Wall Follow Package**
   
Inside the driver stack container, navigate to your ROS 2 workspace  ~/f1tenth_ws/src and create a new package:

```bash 
cd ~/f1tenth_ws/src
ros2 pkg create wall_follow_package --build-type ament_python --dependencies rclpy sensor_msgs std_msgs
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

### **5️⃣ Implement the Wall Follow Node**
You’ll create a wall follow node that listens to LiDAR data (/scan) and publishes a steering commands to stay near a wall

Example: Python Wall follow Node (wall_follow_node.py)
Create a file inside wall_follow/wall_follow_node.py:

```python
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers

        # TODO: set PID gains
        # self.kp = 
        # self.kd = 
        # self.ki = 

        # TODO: store history
        # self.integral = 
        # self.prev_error = 
        # self.error = 

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        return 0.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        return 0.0

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = 0.0 # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **6️⃣ Make It Executable**
Modify setup.py inside safety_package:

```python
entry_points={
    'console_scripts': [
        'wall_follow_node = wall_follow_package.wall_follow_node:main',
    ],
},
```

### **7️⃣ Build & Run**
Run the following:

```bash
cd ~/f1tenth_ws
colcon build --packages-select wall_follow_package
source install/setup.bash
ros2 run wall_follow_package wall_follow_node
```






