---
sort: 2
---

# User PC

## Installing ROS2

To interface with the robot, it is recommended to use a remote PC running the appropriate version of Ubuntu Desktop, with ROS2 installed.

{% tabs installation %}
{% tab installation galactic %}

Required OS: [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)

Follow [these instructions](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) to install ROS2 Galactic on your PC.

Once ROS2 is installed, install `turtlebot4_desktop`:

```bash
sudo apt update && sudo apt install ros-galactic-turtlebot4-desktop
```

{% endtab %}
{% tab installation humble %}

Required OS: [Ubuntu 22.04](https://releases.ubuntu.com/22.04/)

Follow [these instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS2 Humble on your PC.

Once ROS2 is installed, install `turtlebot4_desktop`:

```bash
sudo apt update && sudo apt install ros-humble-turtlebot4-desktop
```

{% endtab %}
{% endtabs %}

## Simple Discovery

If you are using [Simple Discovery](./networking.md#simple-discovery), you may want to create a `setup.bash` file which you can source to apply your ROS2 settings. add the following lines to your `~/.bashrc` file. This will make sure that any time you open a new terminal, the correct ROS2 settings are applied.

{% tabs simple %}
{% tab simple galactic %}

Create a file called `setup.bash` in a convenient location:

```bash
sudo mkdir /etc/turtlebot4/
sudo touch /etc/turtlebot4/setup.bash
```

Add the following lines to it with your favourite text editor:

```bash
source /opt/ros/galactic/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

```tip
Source your workspaces and add any other environment variables to this file.
```

Finally, add the following line in `~/.bashrc` to apply the settings to every new terminal:

```bash
source /etc/turtlebot4/setup.bash
```

```note
Call `source ~/.bashrc` to apply these settings to your current terminal.
```

{% endtab %}
{% tab simple humble %}

Create a file called `setup.bash` in a convenient location:

```bash
sudo mkdir /etc/turtlebot4/
sudo touch /etc/turtlebot4/setup.bash
```

Add the following lines to it with your favourite text editor:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

```tip
Source your workspaces and add any other environment variables to this file.
```

Finally, add the following line in `~/.bashrc` to apply the settings to every new terminal:

```bash
source /etc/turtlebot4/setup.bash
```

```note
Call `source ~/.bashrc` to apply these settings to your current terminal.
```

{% endtab %}
{% endtabs %}

### CycloneDDS

CycloneDDS needs to be configured on the user PC in order to see the robot topics properly. CycloneDDS is configured in an XML file, and that configuration should be applied to the `CYCLONEDDS_URI` environment variable. An example XML file is available [here](https://github.com/turtlebot/turtlebot4_setup/blob/galactic/conf/cyclonedds_pc.xml). Download this file to your PC by calling:

```bash
wget https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/galactic/conf/cyclonedds_pc.xml
```

```note
The `DontRoute` setting is required to see the Create 3 topics. 
```

Move the xml file to a convenient location:

```bash
sudo mv cyclonedds_pc.xml /etc/turtlebot4/
```

Add this line to your `/etc/turtlebot4/setup.bash` file to automatically configure CycloneDDS each time you open a new terminal:

```bash
export CYCLONEDDS_URI=/etc/turtlebot4/cyclonedds_pc.xml
```

```note
Call `source ~/.bashrc` to apply these settings to your current terminal.
```

For more CycloneDDS configuration options, visit the [CycloneDDS documentation](https://github.com/eclipse-cyclonedds/cyclonedds#run-time-configuration).


## Discovery Server

The discovery server requires some additional configuration. Because the Create 3 is not on the same network as the user PC, an IP route has to be added to the Create 3, through the Raspberry Pi. This route must be up at all times to maintain a connection between the PC and the Create 3. Additionally, every device in the system must use `rmw_fastrtps_cpp` as their DDS and must define the `ROS_DISCOVERY_SERVER` environment variable to inform FastDDS of the IP and port of the server.

```note
The Raspberry Pi must be connected to Wi-Fi before proceeding. Follow the [Robot Setup](./robot.html#robot) instructions.
```

{% tabs discovery %}
{% tab discovery galactic %}

Each remote PC that will be communicating with the TurtleBot 4 will have to add an IP route for the Create 3, as well as configure the ROS2 environment to use the RPi4 as the discovery server. The IP route will be added using a service so that it persists through reboots.

Get and install discovery files:
```
git clone https://github.com/turtlebot/turtlebot4_setup.git -b galactic && \
sudo mv turtlebot4_setup/ip_route/ip_route.service /etc/systemd/system/ && \
sudo mv turtlebot4_setup/ip_route/ip_route.sh /usr/local/sbin/ && \
sudo mkdir /etc/turtlebot4_discovery/ && \
sudo mv turtlebot4_setup/turtlebot4_discovery/discovery_super_client.xml /etc/turtlebot4_discovery/ && \
sudo mv turtlebot4_setup/turtlebot4_discovery/setup.bash /etc/turtlebot4_discovery/ && \
rm turtlebot4_setup -rf
```
Get the RPi4 Wi-Fi IP address and replace the following values with it:
- In `/usr/local/sbin/ip_route.sh`, replace `10.42.0.1`
- In `/etc/turtlebot4_discovery/discovery_super_client.xml`, replace `127.0.0.1`
- In `/etc/turtlebot4_discovery/setup.bash`, replace `127.0.0.1`

Start the ip route service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable ip_route.service
sudo systemctl start ip_route.service
```

Add this line to .bashrc: `source /etc/turtlebot4_discovery/setup.bash`

Source .bashrc to apply settings:
```bash
source ~/.bashrc
```

Then, run:

```bash
ros2 daemon stop; ros2 daemon start
```

to restart the ROS2 daemon.

You should now be able to see the Raspberry Pi and Create 3 topics:

```bash
ros2 topic list
```

{% endtab %}
{% tab discovery humble %}

A convenient script to configure the user PC exists in the [turtlebot4_setup](https://github.com/turtlebot/turtlebot4_setup/blob/humble/turtlebot4_discovery/configure_discovery.sh) repo.

To download and run the script, call:

```bash
wget -qO - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/turtlebot4_discovery/configure_discovery.sh | bash <(cat) </dev/tty
```

You will be prompted for a few settings:

- The Raspberry Pi Wi-Fi IP address
- The Discovery Server IP address (Raspberry Pi IP by default)
- The Discovery Server port (11811 by default)
- The `ROS_DOMAIN_ID` of your robot (0 by default)

Once you have entered these values, the script will perform the following:

- Check if an IP route already exists from a previous call of the script. If it does, delete the route.
- Create a directory called `/etc/turtlebot4_discovery/`
- Install an IP route service for the given Raspberry Pi IP address.
- Install a FastDDS super client [profile](https://github.com/turtlebot/turtlebot4_setup/blob/humble/turtlebot4_discovery/fastdds_discovery_super_client.xml) to `/etc/turtlebot4_discovery/`.
- Install a [setup.bash](https://github.com/turtlebot/turtlebot4_setup/blob/humble/turtlebot4_discovery/setup.bash) file to `/etc/turtlebot4_discovery/` with the required configurations.
- Add the line `source /etc/turtlebot4_discovery/setup.bash` to your `~/.bashrc` file.
- Enable and start the IP route service.

When the script has run, call:
```bash
source ~/.bashrc
```
to apply the new settings.

Check that the IP route has been applied by calling:

```bash
ip route
```

You should see an entry like:

```bash
192.168.186.0/24 via 10.0.0.121 dev wlp0s20f3
```

where `10.0.0.121` would be your Raspberry Pi IP, and `wlp0s20f3` would be your Wi-Fi interface.

```note
If you cannot see the route, make sure you are able to ping your Raspberry Pi from your PC. 
Also, try restarting the IP route service by calling `sudo systemctl restart ip_route.service`.
```

Then, run:

```bash
ros2 daemon stop; ros2 daemon start
```

to restart the ROS2 daemon.

You should now be able to see the Raspberry Pi and Create 3 topics:

```bash
ros2 topic list
```

{% endtab %}
{% endtabs %}