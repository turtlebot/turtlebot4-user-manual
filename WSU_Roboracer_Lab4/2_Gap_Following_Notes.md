---
sort: 2
---

# Gap Following NOTES

To set up your gap following package within the driver stack container follow these steps:

### **1️⃣ Create the Gap Follow Package**

Inside the driver stack container, navigate to your ROS 2 workspace `~/f1tenth_ws/src` and create a new package:

```bash
cd ~/f1tenth_ws/src
ros2 pkg create gap_follow --build-type ament_python --dependencies rclpy sensor_msgs std_msgs ackermann_msgs
```

```note
Dependencies:
- rclpy (ROS Client Library for Python)
- sensor_msgs (Standard Sensor Messages)
- std_msgs (Standard ROS Messages)
- ackermann_msgs (Ackermann Drive Messages)
```

### **2️⃣ Modify package.xml**

Ensure `package.xml` includes dependencies such as `rclpy`, `sensor_msgs`, `std_msgs`, and `ackermann_msgs`. Open `package.xml` and verify:

```xml
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>ackermann_msgs</depend>
```

If you're using C++ instead of Python, also ensure you have:

```xml
  <depend>rclcpp</depend>
  <depend>tf2_ros</depend>
  <depend>geometry_msgs</depend>
```

### **3️⃣ Modify CMakeLists.txt (If Using C++)**
If you're using C++, modify `CMakeLists.txt` to include:

```bash
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
```

Ensure the `add_executable` or `ament_target_dependencies` includes these necessary dependencies.

### **4️⃣ Install Dependencies Using rosdep**
Run the following to install missing dependencies:

```bash
cd ~/f1tenth_ws
rosdep install --from-paths src --ignore-src -r -y
```

### **5️⃣ Implement the Gap Follow Node**
You’ll create a gap follow node that processes LiDAR data (`/scan`) to find the largest gap and publishes steering commands to navigate through it.

**Example:** Python Gap Follow Node (`gap_follow_node.py`)
Create a file inside `gap_follow/gap_follow_node.py`:

```python
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class GapFollow(Node):
    """
    Implement Gap Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('gap_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive

    def preprocess_lidar(self, ranges):
        """
        Preprocess LiDAR ranges to handle NaNs, infinities, and limited ranges.

        Args:
            ranges: Array of LiDAR distances

        Returns:
            processed_ranges: Cleaned ranges
        """
        # TODO: implement preprocessing
        return ranges

    def find_largest_gap(self, ranges):
        """
        Find the largest gap in the processed LiDAR data.

        Args:
            ranges: Cleaned LiDAR ranges

        Returns:
            start_idx, end_idx: indices of the largest gap
        """
        # TODO: implement
        return 0, 0

    def calculate_best_point(self, ranges, start_idx, end_idx):
        """
        Find the best point within the largest gap to drive towards.

        Args:
            ranges: Cleaned LiDAR ranges
            start_idx, end_idx: indices of the largest gap

        Returns:
            best_point_idx: Index of the target point
        """
        # TODO: implement
        return 0

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculates the best driving direction.

        Args:
            msg: Incoming LaserScan message
        """
        processed_ranges = self.preprocess_lidar(msg.ranges)
        start, end = self.find_largest_gap(processed_ranges)
        best_point = self.calculate_best_point(processed_ranges, start, end)
        
        drive_msg = AckermannDriveStamped()
        # TODO: compute steering angle and velocity
        # drive_msg.drive.steering_angle = 
        # drive_msg.drive.speed = 

        # TODO: Publish drive message


def main(args=None):
    rclpy.init(args=args)
    print("GapFollow Initialized")
    gap_follow_node = GapFollow()
    rclpy.spin(gap_follow_node)

    gap_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **6️⃣ Make It Executable**
Modify `setup.py` inside `gap_follow`:

```python
entry_points={
    'console_scripts': [
        'gap_follow_node = gap_follow.gap_follow_node:main',
    ],
},
```

### **7️⃣ Build & Run**
Run the following:

```bash
cd ~/f1tenth_ws
colcon build --packages-select gap_follow
source install/setup.bash
ros2 run gap_follow gap_follow_node
```