---
sort: 6
---

# Multiple robots

By default, each TurtleBot 4 will use the same [topic, action, and service names](../software/turtlebot4_packages.md#messages) for communication. If we connect two default TurtleBot 4's to the same network, the topics from both robots will communicate with each other and cause unwanted behaviours.

This tutorial will cover the two main methods for running multiple TurtleBot 4's on a single network.

## ROS_DOMAIN_ID

The **ROS_DOMAIN_ID** is an environment variable used in ROS2 to change the ports that ROS2 processes use for communication. This effectively allows us to separate different ROS2 processes from communicating with each other on the same network. For a detailed explanation, click [here](https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html).

We can leverage this by assigning a unique **ROS_DOMAIN_ID** to each of our robots. This will ensure that communication between robots does not occur. The **ROS_DOMAIN_ID** can be set to any value between 0 and 101, inclusively.

### Setting the environment variable

There are four locations in which the **ROS_DOMAIN_ID** environment variable must be set:
 - Create® 3
 - RPi4 Terminal
 - RPi4 Robot Upstart Job
 - User PC

For RPi4 images v0.1.3 or higher, a script `ros_config.sh` has been installed which will conveniently set the **ROS_DOMAIN_ID** in the first 3 locations. If you are using an earlier version, you can copy the [script](https://github.com/turtlebot/turtlebot4_setup/blob/roni-kreinin/domain_id/scripts/ros_config.sh) to `/usr/local/bin/` on your RPi4.

Simply call `ros_config.sh` on your RPi4 and follow the instructions. Note that this script can also be used to switch between `fastrtps` and `cyclonedds` middlewares.

<figure class="aligncenter">
    <img src="media/ros_config.png" alt="ROS Config" style="width: 60%"/>
    <figcaption>Using the ros_config.sh script</figcaption>
</figure>

Once the settings are confirmed, the script will apply the changes to the Create® 3 and restart the Create® application. This may take about a minute or so. The script will also uninstall the TurtleBot 4 Robot Upstart job, and reinstall it with the new settings. Finally, it will add the **ROS_DOMAIN_ID** to the `~/.bashrc` file of the RPi4. To apply the change to your current RPi4 terminal you will need to call `source ~/.bashrc`.

Once the Create® 3 application has restarted, try calling `ros2 topic list` on both your PC and the RPi4 to ensure that all topics are visible.

#### Create® 3

To manually set the **ROS_DOMAIN_ID** of the Create® 3, go to the [webserver](../overview/quick_start.md#access-the-webserver-v013-or-higher). Navigate to Application->Configuration. Set the Domain ID and click 'Save'. Then restart the application.

#### RPi4 Terminal

To manually set the **ROS_DOMAIN_ID** on the RPi4 terminal, simply call `export ROS_DOMAIN_ID=#`, replacing `#` with your ID number. Note that this will only be applied in your current terminal. To apply this environment variable to all new terminals, add this line to your `~/.bashrc` file.

#### RPi4 Robot Upstart Job

To change the **ROS_DOMAIN_ID** of the TurtleBot 4 Robot Upstart job, you will need to reinstall the job.

First, stop the `turtlebot4` service:

```bash
sudo systemctl stop turtlebot4.service
```

Then, uninstall it:

```bash
uninstall.py
```

Now you can install it again:

##### v0.1.3 or higher

```bash
install.py <model> --domain #
```

##### v0.1.2

```bash
install.py <model> #
```

```note
Replace <model> with your TurtleBot 4 model (standard or lite) and # with your ROS_DOMAIN_ID
```

#### User PC

The **ROS_DOMAIN_ID** can be set on the User PC in the same way as the [RPi4 Terminal](#rpi4-terminal).

## Namespacing

Namespacing support is coming soon.