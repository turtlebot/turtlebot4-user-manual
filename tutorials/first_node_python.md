---
sort: 3
---

# Creating your first node (Python)

This tutorial will go through the steps of creating a ROS 2 package and writing a ROS 2 node in Python. For a C++ example, click [here](./first_node_cpp.html#creating-your-first-node-c). 

{% tabs navigation %}
{% tab navigation galactic %}

These steps are similar to the [ROS 2 Tutorial](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html), but focus on interacting with the TurtleBot 4. For source code, click [here](https://github.com/turtlebot/turtlebot4_tutorials/tree/galactic/turtlebot4_cpp_tutorials).

{% endtab %}
{% tab navigation humble %}

 These steps are similar to the [ROS 2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html), but focus on interacting with the TurtleBot 4. For source code, click [here](https://github.com/turtlebot/turtlebot4_tutorials/tree/humble/turtlebot4_cpp_tutorials).

{% endtab %}
{% endtabs %}

```note
You can follow this tutorial on either the Raspberry Pi of your TurtleBot 4, or your PC.
```

## Create a workspace

If you do not already have a workspace, open a terminal and create one in the directory of your choice:

```bash
mkdir ~/turtlebot4_ws/src -p
```

## Create a package and node

{% tabs package %}
{% tab package galactic %}

You will need to create a ROS 2 package to hold your files. For this tutorial, we will create a package called `turtlebot4_python_tutorials` with a node called `turtlebot4_first_python_node`.

```bash
source /opt/ros/galactic/setup.bash
cd ~/turtlebot4_ws/src
ros2 pkg create --build-type ament_python --node-name turtlebot4_first_python_node turtlebot4_python_tutorials
```

{% endtab %}
{% tab package humble %}

You will need to create a ROS 2 package to hold your files. For this tutorial, we will create a package called `turtlebot4_python_tutorials` with a node called `turtlebot4_first_python_node`.

```bash
source /opt/ros/humble/setup.bash
cd ~/turtlebot4_ws/src
ros2 pkg create --build-type ament_python --node-name turtlebot4_first_python_node turtlebot4_python_tutorials
```

{% endtab %}
{% endtabs %}

This will create a `turtlebot4_python_tutorials` folder and populate it with a basic "Hello World" node, as well as the setup and package.xml files required for a ROS 2 Python package.

## Write your node

The next step is to start coding. For this tutorial, our goal will be to use the Create® 3 interface button 1 to change the colour of the Create® 3 lightring. Open up the "Hello World" `.py` file located at `~/turtlebot4_ws/src/turtlebot4_python_tutorials/turtlebot4_python_tutorials/turtlebot4_first_python_node.py` in your favourite text editor.

### Add your dependencies

For this tutorial, we will need to use the `rclpy` and `irobot_create_msgs` packages. The `rclpy` package allows us to create ROS 2 nodes and gives us full access to all the base ROS 2 functionality in Python. The `irobot_create_msgs` package gives us access to the custom messages used by the Create® 3 for reading the button presses and controlling the lightring.

In package.xml, add these lines under `<buildtool_depend>ament_cmake</buildtool_depend>`:

```xml
<depend>rclpy</depend>
<depend>irobot_create_msgs</depend>
```

In your `.py` file, import these packages:

```py
from irobot_create_msgs.msg import InterfaceButtons, LightringLeds

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
```

### Create a class

Now that the dependencies are set, we can create a class that inherits from the `rclpy.Node` class. We will call this class `TurtleBot4FirstNode`.

```py
class TurtleBot4FirstNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_first_python_node')
```

Notice that our class calls the `super()` constructor and passes it the name of our node, `turtlebot4_first_python_node`. 

We can now create our node in the `main` function and spin it. Since our node is empty, the node will be created but it won't do anything.

```py
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4FirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Subscribe to the Create® 3 interface buttons

Our next step is to subscribe to the Create® 3 interface buttons topic to receive button presses.

```note
The Create® 3 interface buttons will still execute their standard operations. Pressing button 1 will direct the robot to dock (if it is currently undocked) and pressing button 2 will direct the robot to undock (if it is currently docked).
```

We will need to create a `rclpy.Subscription` as well as a callback function for the subscription. The callback function will be called every time we receive a message on the interface buttons topic.

```py
class TurtleBot4FirstNode(Node):
    lights_on_ = False

    def __init__(self):
        super().__init__('turtlebot4_first_python_node')

        # Subscribe to the /interface_buttons topic
        self.interface_buttons_subscriber = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.interface_buttons_callback,
            qos_profile_sensor_data)
    
    # Interface buttons subscription callback
    def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
```

Notice that the `interface_buttons_subscriber` uses the [InterfaceButtons](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/InterfaceButtons.msg) message type, and the quality of service is `qos_profile_sensor_data`. These parameters must match the topic, otherwise the subscription will fail. If you are unsure what message type or QoS a topic is using, you can use the ROS 2 CLI to find this information.

Call `ros2 topic info /<topic> --verbose` to get the full details.

<figure class="aligncenter">
    <img src="media/topic_info.png" alt="Topic Info" style="width: 60%"/>
    <figcaption>ROS 2 topic information</figcaption>
</figure>

### Test Create® 3 Button 1

Now that we are subscribed, lets test out our node by printing a message every time button 1 is pressed.

Edit the `interface_buttons_callback` function to look like this:

```python
# Interface buttons subscription callback
def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
    # Button 1 is pressed
    if create3_buttons_msg.button_1.is_pressed:
        self.get_logger().info('Button 1 Pressed!')
```

Now every time we receive a message on the `/interface_buttons` topic we will check if button 1 is pressed, and if it is then the node will print a message.

To test this out, we will need to build our package using `colcon`:

```bash
cd ~/turtlebot4_ws
colcon build --symlink-install --packages-select turtlebot4_python_tutorials
source install/local_setup.bash
```

The `--symlink-install` allows us to install a symbolic link to our Python script, rather than a copy of the script. This means that any changes we make to the script will be applied to the installed script, so we don't need to rebuild the package after each change.

The `--packages-select` flag allows you to enter any number of packages that you want to build, in case you don't want to build all packages in your workspace.

Now, try running the node:

```bash
ros2 run turtlebot4_python_tutorials turtlebot4_first_python_node
```

When you run it, nothing will happen until you press button 1 on your TurtleBot 4.

Press the button, and you should see this message in your terminal:

```
[INFO] [1652384338.145094927] [turtlebot4_first_python_node]: Button 1 Pressed!
```

```tip
Printing messages like this is a great way to debug your code.
```

### Create a lightring publisher

Now that we can receive a button press, lets create a lightring publisher.

```py
class TurtleBot4FirstNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_first_python_node')

        # Subscribe to the /interface_buttons topic
        self.interface_buttons_subscriber = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.interface_buttons_callback,
            qos_profile_sensor_data)

        # Create a publisher for the /cmd_lightring topic
        self.lightring_publisher = self.create_publisher(
            LightringLeds,
            '/cmd_lightring',
            qos_profile_sensor_data)
```

```note
The Lightring publisher uses the [LightringLeds](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/LightringLeds.msg) message type.
```

Next, lets create a function that will populate a `LightringLeds` message, and publish it.

Add this code below your `interface_buttons_callback` function:

```py
def button_1_function(self):
    # Create a ROS 2 message
    lightring_msg = LightringLeds()
    # Stamp the message with the current time
    lightring_msg.header.stamp = self.get_clock().now().to_msg()

    # Override system lights
    lightring_msg.override_system = True

    # LED 0
    lightring_msg.leds[0].red = 255
    lightring_msg.leds[0].blue = 0
    lightring_msg.leds[0].green = 0

    # LED 1
    lightring_msg.leds[1].red = 0
    lightring_msg.leds[1].blue = 255
    lightring_msg.leds[1].green = 0

    # LED 2
    lightring_msg.leds[2].red = 0
    lightring_msg.leds[2].blue = 0
    lightring_msg.leds[2].green = 255

    # LED 3
    lightring_msg.leds[3].red = 255
    lightring_msg.leds[3].blue = 255
    lightring_msg.leds[3].green = 0

    # LED 4
    lightring_msg.leds[4].red = 255
    lightring_msg.leds[4].blue = 0
    lightring_msg.leds[4].green = 255

    # LED 5
    lightring_msg.leds[5].red = 0
    lightring_msg.leds[5].blue = 255
    lightring_msg.leds[5].green = 255

    # Publish the message
    self.lightring_publisher.publish(lightring_msg)
```

This function creates a `LightringLeds` message and populates the parameters.

We first stamp the message with the current time:

```py
lightring_msg.header.stamp = self.get_clock().now().to_msg()
```

Then we set the `override_system` parameter to `True` so that our command overrides whatever commands the Create® 3 is sending to the lightring.

```py
lightring_msg.override_system = True
```

Next, we populate the 6 LEDs in the `leds` array with whatever colours we want.

```py
# LED 0
lightring_msg.leds[0].red = 255
lightring_msg.leds[0].blue = 0
lightring_msg.leds[0].green = 0

# LED 1
lightring_msg.leds[1].red = 0
lightring_msg.leds[1].blue = 255
lightring_msg.leds[1].green = 0

# LED 2
lightring_msg.leds[2].red = 0
lightring_msg.leds[2].blue = 0
lightring_msg.leds[2].green = 255

# LED 3
lightring_msg.leds[3].red = 255
lightring_msg.leds[3].blue = 255
lightring_msg.leds[3].green = 0

# LED 4
lightring_msg.leds[4].red = 255
lightring_msg.leds[4].blue = 0
lightring_msg.leds[4].green = 255

# LED 5
lightring_msg.leds[5].red = 0
lightring_msg.leds[5].blue = 255
lightring_msg.leds[5].green = 255
```

```tip
Each RGB value can be set between 0 and 255. You can look up the RGB value of any color and set it here.
```

Finally, we publish the message.

```py
self.lightring_publisher.publish(lightring_msg)
```

### Publish the lightring command with a button press

Now we can connect our interface button subscription to our lightring publisher. Simply call `button_1_function` inside the `interface_buttons_callback`.

```py
# Interface buttons subscription callback
def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
    # Button 1 is pressed
    if create3_buttons_msg.button_1.is_pressed:
        self.get_logger().info('Button 1 Pressed!')
        self.button_1_function()
```

Test this out by running the node like before. 

Press button 1 and the lightring light should look like this:

<figure class="aligncenter">
    <img src="media/lightring.jpg" alt="Lightring" style="width: 50%"/>
    <figcaption>Lightring colours controlled with the press of a button!</figcaption>
</figure>

### Toggle the lightring

You will notice that once you have set the lightrings LEDs they will remain like that forever. Lets make the button toggle the light on or off each time we press it.

Add a boolean to keep track of the light state:

```py
class TurtleBot4FirstNode(Node):
    lights_on_ = False

    def __init__(self):
```

And modify `button_1_function` to toggle the light:

```py
# Perform a function when Button 1 is pressed
def button_1_function(self):
    # Create a ROS 2 message
    lightring_msg = LightringLeds()
    # Stamp the message with the current time
    lightring_msg.header.stamp = self.get_clock().now().to_msg()

    # Lights are currently off
    if not self.lights_on_:
        # Override system lights
        lightring_msg.override_system = True

        # LED 0
        lightring_msg.leds[0].red = 255
        lightring_msg.leds[0].blue = 0
        lightring_msg.leds[0].green = 0

        # LED 1
        lightring_msg.leds[1].red = 0
        lightring_msg.leds[1].blue = 255
        lightring_msg.leds[1].green = 0

        # LED 2
        lightring_msg.leds[2].red = 0
        lightring_msg.leds[2].blue = 0
        lightring_msg.leds[2].green = 255

        # LED 3
        lightring_msg.leds[3].red = 255
        lightring_msg.leds[3].blue = 255
        lightring_msg.leds[3].green = 0

        # LED 4
        lightring_msg.leds[4].red = 255
        lightring_msg.leds[4].blue = 0
        lightring_msg.leds[4].green = 255

        # LED 5
        lightring_msg.leds[5].red = 0
        lightring_msg.leds[5].blue = 255
        lightring_msg.leds[5].green = 255
    # Lights are currently on
    else:
        # Disable system override. The system will take back control of the lightring.
        lightring_msg.override_system = False

    # Publish the message
    self.lightring_publisher.publish(lightring_msg)
    # Toggle the lights on status
    self.lights_on_ = not self.lights_on_
```

Now the Create® 3 will regain control of the lightring if we press button 1 again.

### Your first Python Node

You have finished writing your first Python node! The final `.py` file should look like this:

```py
from irobot_create_msgs.msg import InterfaceButtons, LightringLeds

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class TurtleBot4FirstNode(Node):
    lights_on_ = False

    def __init__(self):
        super().__init__('turtlebot4_first_python_node')

        # Subscribe to the /interface_buttons topic
        self.interface_buttons_subscriber = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.interface_buttons_callback,
            qos_profile_sensor_data)

        # Create a publisher for the /cmd_lightring topic
        self.lightring_publisher = self.create_publisher(
            LightringLeds,
            '/cmd_lightring',
            qos_profile_sensor_data)

    # Interface buttons subscription callback
    def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
        # Button 1 is pressed
        if create3_buttons_msg.button_1.is_pressed:
            self.get_logger().info('Button 1 Pressed!')
            self.button_1_function()

    # Perform a function when Button 1 is pressed
    def button_1_function(self):
        # Create a ROS 2 message
        lightring_msg = LightringLeds()
        # Stamp the message with the current time
        lightring_msg.header.stamp = self.get_clock().now().to_msg()

        # Lights are currently off
        if not self.lights_on_:
            # Override system lights
            lightring_msg.override_system = True

            # LED 0
            lightring_msg.leds[0].red = 255
            lightring_msg.leds[0].blue = 0
            lightring_msg.leds[0].green = 0

            # LED 1
            lightring_msg.leds[1].red = 0
            lightring_msg.leds[1].blue = 255
            lightring_msg.leds[1].green = 0

            # LED 2
            lightring_msg.leds[2].red = 0
            lightring_msg.leds[2].blue = 0
            lightring_msg.leds[2].green = 255

            # LED 3
            lightring_msg.leds[3].red = 255
            lightring_msg.leds[3].blue = 255
            lightring_msg.leds[3].green = 0

            # LED 4
            lightring_msg.leds[4].red = 255
            lightring_msg.leds[4].blue = 0
            lightring_msg.leds[4].green = 255

            # LED 5
            lightring_msg.leds[5].red = 0
            lightring_msg.leds[5].blue = 255
            lightring_msg.leds[5].green = 255
        # Lights are currently on
        else:
            # Disable system override. The system will take back control of the lightring.
            lightring_msg.override_system = False

        # Publish the message
        self.lightring_publisher.publish(lightring_msg)
        # Toggle the lights on status
        self.lights_on_ = not self.lights_on_


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4FirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```
