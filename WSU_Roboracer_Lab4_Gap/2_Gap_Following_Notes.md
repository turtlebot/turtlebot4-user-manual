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
    Gap Follow class to steer the car toward the largest navigable space.
    """

    def __init__(self):
        super().__init__('gap_follow_node')

        lidarscan_topic = '/scan'   # Topic for incoming LaserScan messages
        drive_topic = '/drive'      # Topic to publish drive commands

        # TODO: Subscribe to the LiDAR topic
        # TODO: Create a publisher for AckermannDriveStamped messages

        # Optional: Initialize other class variables if needed

    def preprocess_lidar(self, ranges):
        """
        Preprocess the LiDAR scan array. Preprocessing steps may include:
            
            1. Clipping maximum range values
            2. Handling NaN or inf values
            3. (Optional) Smoothing (e.g., moving average)

        Args:
            ranges (list): Raw LiDAR distance readings

        Returns:
            np.array: Cleaned and processed LiDAR distances
        """
        proc_ranges = np.array(ranges)

        # TODO: 1. Clip values that are too far (e.g., > 3 meters) to a maximum distance
        # Hint: Use np.clip()

        # TODO: 2. Replace NaN values with a maximum valid distance
        # Hint: Use np.isnan()

        # TODO: 2. Replace infinite values with a maximum valid distance
        # Hint: Use np.isinf()

        # Optional TODO: 3. Apply smoothing (e.g., moving average) if desired

        return proc_ranges

    def create_safety_bubble(self, proc_ranges, bubble_radius=0.5):
        """
        Create a safety bubble around the closest obstacle by zeroing out nearby points.

        Args:
            proc_ranges (np.array): Processed LiDAR ranges
            bubble_radius (float): Radius of the bubble in meters

        Returns:
            np.array: Updated LiDAR ranges with the bubble applied
        """
        # TODO: Get the angle increment from the incoming scan (store it earlier)

        # TODO: Find the index of the closest obstacle

        # TODO: Calculate how many indices the bubble should cover

        # TODO: Set the points inside the bubble to zero

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """
        Find the largest sequence of valid points (non-obstacles).

        Args:
            free_space_ranges (np.array): Processed LiDAR data

        Returns:
            (start_idx, end_idx): Start and end indices of the largest navigable gap
        """
        # TODO: Initialize variables to keep track of the maximum gap
        # (example: max_start_idx, max_end_idx, max_gap_size)

        # TODO: Initialize variables to track the current gap while iterating
        # (example: current_start_idx)

        # TODO: Loop through the free_space_ranges array
        # Hint: Look for sequences of non-zero points (non-obstacles)

            # TODO: If you encounter a free space (non-zero distance):
            # - Start or continue counting the current gap

            # TODO: If you encounter an obstacle (zero distance):
            # - Check if the current gap is larger than the previous max gap
            # - Update max gap variables if needed
            # - Reset current gap tracking

        # TODO: After the loop, check if the last gap is the largest

        # TODO: Return the start and end indices of the largest gap
        return None

    def find_best_point(self, ranges, start_idx, end_idx):
        """
        Select the best point within the largest gap to aim toward.

        Args:
            ranges (np.array): Processed LiDAR data
            start_idx (int): Starting index of the gap
            end_idx (int): Ending index of the gap

        Returns:
            best_point_idx (int): Index of the selected steering point
        """
        # TODO: Extract the subset of ranges between start_idx and end_idx
        # (Hint: slice the array)

        # TODO: Find the index within the gap that has the maximum distance
        # (This would represent the farthest safe point)

        # TODO: Adjust the best point index relative to the original ranges
        # (Remember: the sliced array starts at start_idx)

        # TODO: Return the corrected best point index
        return None

    def scan_callback(self, data: LaserScan):
        """
        Callback triggered by incoming LaserScan data.

        Workflow:
            1. Preprocess scan data
            2. Create a safety bubble around obstacles
            3. Find the maximum navigable gap
            4. Find the best point within the gap
            5. Publish a drive command toward the selected point

        Args:
            data (LaserScan): Incoming LiDAR scan message
        """
        ranges = data.ranges

        # Step 1: Preprocess the scan
        proc_ranges = self.preprocess_lidar(ranges)

        # Step 2: Apply safety bubble
        proc_ranges = self.create_safety_bubble(proc_ranges)

        # Step 3: TODO - Find maximum gap

        # Step 4: TODO - Find best point within gap

        # Step 5: TODO - Publish drive command

    def publish_drive_command(self, steering_angle, speed=1.0):
        """
        Publish a drive command with the desired steering angle and speed.

        Args:
            steering_angle (float): Desired steering angle in radians
            speed (float): Desired speed in meters/second
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed

        # TODO: Publish the drive message

def main(args=None):
    rclpy.init(args=args)
    print("GapFollow Node Initialized")
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