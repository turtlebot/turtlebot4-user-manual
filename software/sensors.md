---
sort: 3
---

# Sensors

## RPLIDAR A1M8

### Connecting

The RPLIDAR connects to the TurtleBot 4 with a micro USB to USB-A cable. The sensor does not require high data throughput, so using a USB 2.0 port is sufficient. 

Once connected, the RPLIDAR should register on the Raspberry PI as a USB device. If the [udev rules](https://github.com/turtlebot/turtlebot4-images/blob/galactic/turtlebot4_setup/udev/turtlebot4.rules) are installed, the RPLIDAR will appear as `/dev/RPLIDAR`. Otherwise it will be `/dev/ttyUSB0`.

To check that the USB device exists, use the command

```bash
ls /dev/RPLIDAR
```

If the device exists, the terminal will echo `/dev/RPLIDAR`.

### Installing

The RPLIDAR drivers are installed by default on all TurtleBot 4's. To manually install, run:

```bash
sudo apt install ros-galactic-rplidar-ros
```

### Running

```bash
ros2 launch turtlebot4_bringup rplidar.launch.py
```

The laserscan will be published to the */scan* topic by default.


## OAK-D

### Connecting

The OAK-D cameras are connected to the Raspberry Pi with a USB-C to USB-A cable. The cameras requires high data throughput so using a USB 3.0 port is highly recommended.

### Installing

The OAK-D drivers are installed by default on all TurtleBot 4's. To manually install, follow the instructions on the DepthAI ROS [github](https://github.com/luxonis/depthai-ros/tree/main#getting-started).

### Running

The default node used by the TurtleBot 4 can be launched:

```bash
ros2 launch turtlebot4_bringup oakd.launch.py
```

Other nodes are available in the DepthAI ROS examples [package](https://github.com/luxonis/depthai-ros-examples/tree/main/depthai_examples/launch).

For example:

```bash
ros2 launch depthai_examples mobile_publisher.launch.py
```

AI examples are available on the DepthAI [github](https://github.com/luxonis/depthai-python). To view the images from these examples you will need to ssh into the robot with a `-X` flag.

```bash
ssh ubuntu@192.168.0.15 -X
```

## Create® 3

The Create® 3 comes with several sensors for safety, object detection, and odometry. For more information on the physical location of the sensors, read the Create® 3 [Hardware Overview](https://iroboteducation.github.io/create3_docs/hw/overview/). Hazards detected by the robot are published to the */hazard_detection* topic, although some sensors also have their own individual topics

### Cliff

The Create® 3 has 4 cliff sensors located on the front half of the robot. These sensors measure the distance from the robot to the ground, and prevent the robot from falling off of cliffs.

### Bumper

The bumper is used by the Create® 3 to detect objects or walls that the robot has run in to. It can trigger reflexes to recoil from the object, or use the information to follow the wall.

### Wheeldrop

The wheeldrop is the spring on which the Create® 3 wheels sit. When the robot is lifted off of the ground, the spring is decompressed and the wheeldrop hazard is activated.

### IR Proximity

The IR proxmity sensors are located on the front of the bumper and are used for the wall follow action. The sensor data can be viewed on the */ir_intensity* topic.

### Slip and Stall

Wheel slip and stall is also detected by the Create® 3. The status can be viewed on the */slip_status* and */stall_status* topics.

### Kidnap

The robot uses a fusion of sensor data to detect when it has been picked up and "kidnapped". Motors will be disabled in this state, and will re-enable when placed on the ground again. The */kidnap_status* topic shows the current kidnap state.