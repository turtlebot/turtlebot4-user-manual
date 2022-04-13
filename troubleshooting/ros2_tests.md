---
sort: 2
---

# ROS2 Tests

Both TurtleBot 4 models have the `turtlebot4_tests` package installed by default. This package provides some tests that can be run from CLI to test basic system functions.

Each test uses a ROS2 topic, action, or service to perform the action. To run Create® 3 tests, the Create® 3 must be connected to the Raspberry Pi over either WiFi or USB-C.

To run the tests, call

```bash
ros2 run turtlebot4_tests ros_tests
```

<figure class="aligncenter">
    <img src="media/ros_tests.png" alt="ROS tests" style="width: 25%"/>
    <figcaption>Running the Light Ring test</figcaption>
</figure>

Test results are saved to `~/turtlebot4_test_results/Y_m_d-H_M_S` where `Y_m_d-H_M_S` is the date and time of the test. A rosbag is also recorded for the duration of the test and saved to the same location.