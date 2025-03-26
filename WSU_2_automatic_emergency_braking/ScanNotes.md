---
sort: 2
---

# Scan Notes

# Understanding LiDAR Scan Data with RPLidar (Turtlebot4)

When working with an RPLidar sensor mounted on the Turtlebot4, it's important to note that the sensor's orientation may differ from standard configurations. Specifically, the front of the LiDAR sensor on the Turtlebot4 is facing towards the right side of the robot.

## Adjusted LiDAR Data Array (ranges)

Due to the mounting orientation, the `ranges` array corresponds to angles starting from the robot's right side (0 degrees) and goes around counterclockwise:

| Direction      | Angle (degrees) | Index Calculation                   |
|----------------|-----------------|-------------------------------------|
| Right (Front of LiDAR) | 0°      | `0`                                 |
| Front (robot's front)  | 90°     | `len(ranges) // 4`                  |
| Left                   | 180°    | `len(ranges) // 2`                  |
| Back                   | 270°    | `3 * len(ranges) // 4`              |

## Example in ROS 2 Python Node

Here's how you'd access LiDAR data directly in front of the Turtlebot4 (which corresponds to 90 degrees in the LiDAR array):

```python
def scan_callback(self, msg: LaserScan):
    # Directly in front of the robot (90 degrees due to mounting)
    front_index = len(msg.ranges) // 4
    front_distance = msg.ranges[front_index]

    self.get_logger().info(f'Distance in front of robot: {front_distance:.2f} meters')

    if front_distance < 0.5:
        self.get_logger().info('Obstacle detected within 0.5 meters in front of the robot.')
```

## Important Notes:
- Always verify your specific robot's LiDAR mounting orientation before interpreting the data.
- The angle increments in the LiDAR message (`angle_increment`) will help you calculate precise angular positions.
- Handle potential edge cases, such as infinite distances (`inf`) or invalid measurements (`NaN`).

This adjusted guide provides a clear reference for interpreting LiDAR data specifically for the Turtlebot4 configuration.

