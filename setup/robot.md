---
sort: 3
---

# Robot

The first step for setting up the TurtleBot 4 is to power it on and connect it to your Wi-Fi network.

## Power on the robot

Place the TurtleBot 4 onto its dock. The green LED on the dock will turn on for a few seconds, and the TurtleBot 4 should power on. Allow the robot some time to boot up.

## Connect the Raspberry Pi to Wi-Fi

### Connect to the Access Point

On the first boot, the Raspberry Pi will enter Access Point (AP) mode which will allow you to connect to it over Wi-Fi. On your PC, connect to the `Turtlebot4` Wi-Fi network. The password is also `Turtlebot4`.

```note
The TurtleBot 4 AP network is a 5GHz network. Your computer will need to support 5GHz Wi-Fi to connect to the network.
```

### SSH into the Raspberry Pi

Once connected, you can SSH into the Raspberry Pi to configure its Wi-Fi. Open a terminal on your PC and call: 

```bash
ssh ubuntu@10.42.0.1
```

Log in using the password `turtlebot4`.

### Connect the Raspberry Pi to your network

Once logged in, configure the Raspberry Pi to connect to your Wi-Fi network.

```tip
Connect the Raspberry Pi to a 5GHz Wi-Fi network for optimal performance.
```

{% tabs wifi %}
{% tab wifi galactic %}

In your SSH session, call:

```bash
sudo wifi.sh -s '<WIFI_SSID>' -p '<WIFI_PASSWORD>' -r <REGULATORY_DOMAIN> && sudo reboot
```

```note
The Regulatory Domain is based on the country you live in. USA: `US`, Canada: `CA`, UK: `GB`, Germany: `DE`, Japan: `JP3`, Spain: `ES`. For a full list, click [here](https://www.arubanetworks.com/techdocs/InstantWenger_Mobile/Advanced/Content/Instant%20User%20Guide%20-%20volumes/Country_Codes_List.htm#regulatory_domain_3737302751_1017918).
```

{% endtab %}
{% tab wifi humble %}

In your SSH session, run the [TurtleBot 4 setup tool](../software/turtlebot4_setup.md#configuration-tools):

```bash
turtlebot4-setup
```

This will start the TurtleBot 4 setup tool. Navigate to the "Wi-Fi Setup" menu and configure your connection. When you have finished, save and apply the settings.

<figure class="aligncenter">
    <img src="media/wifi_setup.gif" alt="Wi-Fi setup" style="width: 100%"/>
    <figcaption>Wi-Fi Setup using the TurtleBot 4 Setup tool</figcaption>
</figure>

```note
Change your Wi-Fi mode to 'Client' when connecting to an existing network.
```

{% endtab %}
{% endtabs %}

### Find the new Raspberry Pi IP

Once the Wi-Fi settings are applied, the Raspberry Pi will reboot and connect to your network. DHCP will assign it a new IP address. On the TurtleBot 4, this IP address will be shown at the top of the display. 

<figure class="aligncenter">
    <img src="media/display_ip2.jpg" alt="IP Address" style="transform:rotate(270deg); width: 20%"/>
    <figcaption>Wi-Fi IP address on a TurtleBot 4</figcaption>
</figure>

For the TurtleBot 4 Lite, you will need to check the `/ip` topic for the new address.

{% tabs ip %}
{% tab ip galactic %}

On your PC, run the following command:
 
```bash
ros2 topic echo /ip
```
{% endtab %}
{% tab ip humble %}

On your PC, run the following command:

```bash
ros2 topic echo /ip
```

{% endtab %}
{% endtabs %}

You should see the IP address printed out in your terminal periodically.

```bash
$ ros2 topic echo /ip
data: 192.168.28.24
---
data: 192.168.28.24
---
```

```note
If you are unable to find the IP address with the previous methods, try logging into your gateway and looking for a device with host name "ubuntu".
```

Once you have found the IP address, you can now SSH back into the robot with it.

```bash
ssh ubuntu@xxx.xxx.xxx.xxx
```

## Updating the TurtleBot 4

It is recommended to update both the Create® 3 and the Raspberry Pi when you first use it to receive the latest bug fixes and improvements.

### Create® 3

Check the [Create® 3 software releases](https://iroboteducation.github.io/create3_docs/releases/overview/) to see if a newer firmware version is available. You can check the firmware version of your Create® 3 by visiting the webserver.

If new firmware is available, download it, then access the [Create® 3 webserver](https://iroboteducation.github.io/create3_docs/webserver/overview/). Go to the <b>Update</b> tab, upload the firmware, then update your robot.

### Raspberry Pi packages

SSH into the Raspberry Pi, then update all packages by calling:

```bash
sudo apt update && sudo apt upgrade
```

## Simple Discovery

To use the TurtleBot 4 with Simple Discovery, the Create® 3 should be connected to Wi-Fi.

### Create® 3 Wi-Fi Setup

Press both Create® 3 button 1 and 2 simultaneously until light ring turns blue.

<figure class="aligncenter">
    <img src="media/create_ap_mode.jpg" alt="Create AP mode" style="width: 70%"/>
    <figcaption>Putting the Create® 3 in AP mode</figcaption>
</figure>

The Create® 3 is now in AP mode. Connect to its Wi-Fi network called 'Create-XXXX'. Then, in a web browser, navigate to `192.168.10.1`. This will open the Create® 3 [webserver](https://iroboteducation.github.io/create3_docs/webserver/overview/). Go to the Connect tab, enter your Wi-Fi SSID and password, and then click 'Connect'.

<figure class="aligncenter">
    <img src="media/create3_connect.png" alt="Create® 3 connect" style="width: 100%"/>
    <figcaption>Connecting the Create® 3 to Wi-Fi</figcaption>
</figure>

Wait for it to connect to Wi-Fi and play a chime. On your PC, run `ros2 topic list` to ensure that the Create® 3 is publishing its topics.

```note
The Create® 3 can only be connected to 2.4 GHz Wi-Fi networks.
```

## Discovery Server

{% tabs discovery %}
{% tab discovery galactic %}

<u><b style="font-size: 20px;">Create® 3</b></u>

The Create® 3 needs to be updated to the latest firmware, have its Wi-Fi disabled, and be configured to use the Raspberry Pi as the discovery server over the USB-C connection.

<b>Setup instructions:</b>
- Update to the [latest firmware](https://iroboteducation.github.io/create3_docs/releases/overview/) using the webserver.
- Once updated, perform a [factory reset](https://iroboteducation.github.io/create3_docs/webserver/about/#:~:text=set%20to%20USB.-,Factory%20Reset,-A%20hyperlink%20to) to disconnect the Create® 3 from any Wi-Fi networks.
- On the webserver, go to Application -> Configuration. Save the following settings:

<figure class="aligncenter">
    <img src="media/create3_discovery.png" alt="Create® 3 discovery" style="width: 100%"/>
    <figcaption>Create® 3 discovery server settings</figcaption>
</figure>

<u><b style="font-size: 20px;">Raspberry Pi</b></u>

The Raspberry Pi needs to enable IP forwarding, configure itself as the discovery server, and reinstall the TurtleBot 4 upstart job with the new configuration. Environment variables will now be declared in `/etc/turtlebot4_discovery/setup.bash`, such that both the terminal and upstart environment will have the same configurations.

<b>Setup instructions:</b>
- SSH into the Raspberry Pi
- Uncomment "net.ipv4.ip_forward=1" in `/etc/sysctl.conf`
- Get and install discovery files:
```
git clone https://github.com/turtlebot/turtlebot4_setup.git -b galactic && \
sudo mv turtlebot4_setup/turtlebot4_discovery /etc/ && \
sudo mv turtlebot4_setup/scripts/install.py /usr/local/bin/ && \
rm turtlebot4_setup -rf
```
- Source other workspaces in `/etc/turtlebot4_discovery/setup.bash` if needed
- Add the following line to .bashrc: `source /etc/turtlebot4_discovery/setup.bash`
- Re-install the upstart job with the updated `install.py` script:

```bash
install.py --discovery on --workspace /etc/turtlebot4_discovery/setup.bash
```

Restart the robot by powering it off with the power button and placing it back on the dock.

```tip
You can move any ROS2 related environment variables such as `ROS_DOMAIN_ID` from `~/.bashrc` to `/etc/turtlebot4_discovery/setup.bash`
```

{% endtab %}
{% tab discovery humble %}

<u><b style="font-size: 20px;">Create® 3</b></u>

The Create® 3 needs to be updated to the latest firmware and have its Wi-Fi disabled.

<b>Setup instructions:</b>
- Update to the [latest firmware](https://iroboteducation.github.io/create3_docs/releases/overview/) using the webserver.
- Once updated, perform a [factory reset](https://iroboteducation.github.io/create3_docs/webserver/about/#:~:text=set%20to%20USB.-,Factory%20Reset,-A%20hyperlink%20to) to disconnect the Create® 3 from any Wi-Fi networks.

<u><b style="font-size: 20px;">Raspberry Pi</b></u>

The Raspberry Pi needs to configure itself as the discovery server, and reinstall the TurtleBot 4 upstart job with the new configuration.

<b>Setup instructions:</b>
- SSH into the Raspberry Pi
- Run the [TurtleBot 4 setup tool](../software/turtlebot4_setup.md#configuration-tools):

```bash
turtlebot4-setup
```
- Enter the <b>Discovery Server</b> menu via <b>ROS Setup</b>.
- Enable the discovery server.
- Leave the IP address as `127.0.0.1`, and the port as `11811`.
- Save the settings, navigate to the main menu, and apply settings.

Restart the robot by powering it off with the power button and placing it back on the dock.

{% endtab %}
{% endtabs %}

## TurtleBot 4 Controller Setup

The TurtleBot 4 comes with an included TurtleBot 4 Controller. It is paired in advance with the Raspberry Pi.

If you wish to manually pair a controller, follow these instructions:

- SSH into the TurtleBot 4

```bash
sudo bluetoothctl --agent=NoInputNoOutput
```

- The `bluetoothd` CLI interface will start.
- Type `scan on` and press enter.
- Press and hold both the home and share buttons on the TurtleBot 4 controller until the light starts blinking.

<figure class="aligncenter">
    <img src="media/controller.jpg" alt="TurtleBot 4 Controller" style="width: 70%"/>
    <figcaption>Putting the TurtleBot 4 in pair mode</figcaption>
</figure>

- In the CLI look for a *Wireless Controller* device to be found. It will have a MAC address similar to `A0:5A:5C:DF:4D:7F`.
- Copy the MAC address.
- In the CLI enter `trust MAC_ADDRESS`, replacing `MAC_ADDRESS` with the controllers address.
- Then, enter `pair MAC_ADDRESS`.
- Finally, enter `connect MAC_ADDRESS`.
- The CLI should report that the controller has been connected and the light on the controller will turn blue.
- Enter `exit` to exit the CLI.

Visit the [Driving Tutorial](../tutorials/driving.md#joystick-teleoperation) to begin driving your TurtleBot 4.

## Access Point Mode

Placing the Raspberry Pi into Access Point mode can be useful when using the robot in an area without Wi-Fi connection.

{% tabs ap %}
{% tab ap galactic %}

SSH into the Raspberry Pi and call:

```bash
sudo wifi.sh -a && sudo reboot
```

The Raspberry Pi will revert back to the original AP mode.

Optionally you can set your own SSID and password with:

```bash
sudo wifi.sh -s '<SSID>' -p '<PASSWORD>' -a && sudo reboot
```

{% endtab %}
{% tab ap humble %}

SSH into the Raspberry Pi and run the [TurtleBot 4 setup tool](../software/turtlebot4_setup.md#configuration-tools):

```bash
turtlebot4-setup
```

Go to <b>Wi-Fi Setup</b> and select <b>Apply Defaults</b>. Optionally you can set your own SSID and password before saving and applying the new settings.

{% endtab %}
{% endtabs %}

```tip
If you are moving your TurtleBot 4 to a new location with a different Wi-Fi network, 
reconfigure the Raspberry Pi to connect to that network beforehand or place it into AP mode. 
Otherwise it will continue trying to connect to your current network.
```

## Recovering the Raspberry Pi

If you entered incorrect Wi-Fi credentials or your Wi-Fi network is down, you will not be able to access the Raspberry Pi over Wi-Fi. To recover from this, you can connect directly to the Raspberry Pi using an ethernet cable. You may need a USB to Ethernet adapter for your PC.

<figure class="aligncenter">
    <img src="media/ethernet.jpg" alt="Ethernet connection" style="width: 80%;"/>
    <figcaption>Connecting to the TurtleBot 4 over Ethernet</figcaption>
</figure>

The Raspberry Pi uses a static IP address for the ethernet interface, `192.168.185.3`. You will need to configure your wired connection to use the same subnet:

- Go to your wired connection settings.
- Set your IPv4 Method to `Manual` and set your static IP. The IP address cannot be the same as the Raspberry Pi.

<figure class="aligncenter">
    <img src="media/static_ip.png" alt="Setting IP" style="width: 70%;"/>
    <figcaption>Configure your PC's wired IP</figcaption>
</figure>

- Click 'Apply'

You can now go to your terminal and SSH into the robot by typing:

```bash
ssh ubuntu@192.168.185.3
```

## Install latest Raspberry Pi image

```warning
Installing a new image on the Raspberry Pi will delete any changes you may have made. Save your changes externally before proceeding.
```

Find the latest TurtleBot 4 Raspberry Pi images at <http://download.ros.org/downloads/turtlebot4/>.

- Download the latest image extract it. 
- Power off your robot and then remove the microSD card from the Raspberry Pi.
- Insert the microSD card into your PC. You may need an adapter.
- Install the imaging tool `dcfldd`

```bash
sudo apt install dcfldd
```
- Identify your SD card

```bash
sudo fdisk -l
```

- The SD card should have a name like `/dev/mmcblk0` or `/dev/sda`.

- If you wish to backup your current image, do so now:

```bash
sudo dd if=/dev/<SD_NAME> of=<IMAGE_PATH> bs=1M
```

```note
SD_NAME is the device name (`mmcblk0`, `sda`, etc.).

IMAGE_PATH is the path to where you want the image saved -- e.g., `~/turtlebot4_images/backup_image`.
```

- Get the SD flash script from `turtlebot4_setup` and flash the SD card:

```bash
wget https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/galactic/scripts/sd_flash.sh
bash sd_flash.sh /path/to/downloaded/image
```
- Follow the instructions and wait for the SD card to be flashed. 
- Remove the SD card from your PC.
- Ensure your Raspberry Pi 4 is not powered on before inserting the flashed SD card.
- Follow the [Robot Setup](#robot) to configure your TurtleBot 4.
