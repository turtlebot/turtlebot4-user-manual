---
sort: 2
---

# Basic CV 


## Find the camera feed

### 2. Turn on Turtlebot 4

### 3. Check for topic from OAK-D



## Steps to view camera feed through code:

### 1. Create a new ROS2 package

```bash
mkdir -p ~/basic_cv_ws/src
cd ~/basic_cv_ws/src
ros2 pkg create basic_cv_pkg --build-type ament_python --dependencies rclpy
```

### 2. Write the node code

Open the package in vscode and add the basic_cv_node.py python file

```bash 
cd ..
code .
```
### 3. Create the a new file under basic_cv_pkg directory called basic_cv_node.py.  

Edit `basic_cv_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'oakd/rgb/preview/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Manipulate the image here (as an example, convert to grayscale)
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Display the image
        cv2.imshow('Camera Image', cv_image)
        cv2.imshow('Grayscale Image', gray_image)
        # cv2.imshow('Color Image', color_image)
        edges = cv2.Canny(cv_image, 100, 200)
        cv2.imshow('Edge Detected Image', edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Modify Setup.py

Add you python node to the setup.py 

```python
...

'console_scripts': [
            'basic_cv_node = basic_cv_pkg.basic_cv_node:main',
        ],
...
```

### 5. Source Humble Bash**

```bash
cd ~/basic_cv_ws
colcon build --symlink-install
source install/setup.bash
source /opt/ros/humble/setup.bash
```

**Run the Node**
```bash
ros2 run basic_cv_pkg basic_cv_node
```