---
sort: 3
---

# Quality of Service (QoS) in ROS 2

Quality of Service (QoS) settings in ROS 2 determine how data flows between publishers and subscribers, especially crucial for sensor data streams such as LiDAR scans. Proper QoS configuration helps ensure reliability and timeliness of data, significantly impacting the performance of robotic applications.

## QoS for Sensor Data

When working with sensor data, such as LiDAR scans from an RPLidar, ROS 2 provides a predefined QoS profile specifically for sensor data called `qos_profile_sensor_data`. This profile helps manage high-frequency data efficiently by:

- **Reliability:** Set to "best effort" to prioritize the latest available data rather than guaranteeing the delivery of every single message.
- **Durability:** Set to "volatile" since only recent data matters for sensors.
- **History Depth:** Ensures the subscriber buffer holds recent messages to reduce the chance of missing important data.

### Using Sensor QoS in Python

Here's how to apply the `qos_profile_sensor_data` to your subscriber:

```python
from rclpy.qos import qos_profile_sensor_data

self.scan_subscriber = self.create_subscription(
    LaserScan,
    '/scan',
    self.scan_callback,
    qos_profile_sensor_data)
```

## Common QoS Profiles in ROS 2

ROS 2 provides several predefined QoS profiles, each tailored to specific types of communication:

- **Default QoS (`qos_profile_default`)**: General-purpose profile.
- **Sensor Data QoS (`qos_profile_sensor_data`)**: Best suited for high-frequency sensor streams.
- **System Default QoS (`qos_profile_system_default`)**: Matches the default behavior of underlying middleware.
- **Parameters QoS (`qos_profile_parameters`)**: Used for parameters and dynamic configurations.

## Considerations When Using QoS

- Always match QoS profiles between publishers and subscribers to avoid communication issues.
- High-frequency data streams (e.g., LiDAR, IMU) benefit significantly from sensor-specific QoS settings.
- For reliable message exchange (commands or state updates), consider using "reliable" QoS settings.

Properly choosing and configuring QoS settings in ROS 2 will help ensure your robotic system functions efficiently and reliably.