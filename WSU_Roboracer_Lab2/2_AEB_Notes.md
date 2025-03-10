---
sort: 2
---

# Lab 2: Automatic Emergency Braking NOTES

To set up your safety package within the driver stack container and prevent the roboracer from colliding with objects in front of it, follow these steps:

1. Create the Safety Package
   
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

2. Modify package.xml

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