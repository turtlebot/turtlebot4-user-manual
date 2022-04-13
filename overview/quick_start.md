---
sort: 3
---

# Quick Start

## WiFi Setup

- Power on the robot by placing it on its dock.

- The Raspberry Pi will boot after a few moments. 

- On the first boot, the Raspberry Pi will enter AP mode which will allow you to connect to it over WiFi.

- On a PC, connect to the `TurtleBot4` WiFi network. The password is also `TurtleBot4`.

- Once connected, you can SSH into the Raspberry Pi to configure its WiFi.

```bash
ssh ubuntu@10.42.0.1
```
- The default password is `turtlebot4`

- In `/usr/local/bin` there will be a script called `wifi.sh` which can be used to configure the Raspberry Pi's WiFi:

```bash
sudo wifi.sh -s "YOUR_WIFI_SSID" -p "YOUR_WIFI_PASSWORD" -r YOUR_REGULATORY_DOMAIN && sudo reboot
```

```note
The Regulatory domain is based on the country you live in. For a full list, click [here](https://www.arubanetworks.com/techdocs/InstantWenger_Mobile/Advanced/Content/Instant%20User%20Guide%20-%20volumes/Country_Codes_List.htm#regulatory_domain_3737302751_1017918).
```

- Your Raspberry Pi will reboot and connect to your WiFi network.
- On your PC, run `ros2 topic list` to ensure that the TurtleBot 4 is publishing its topics.
- Run `ros2 topic echo /ip` to read the new IP of the Raspberry Pi. On the TurtleBot 4 Standard this will also be displayed on the screen.
- You can now SSH into the TurtleBot 4's Raspberry Pi at the new IP and begin using it.

```bash
ssh ubuntu@xxx.xxx.xxx.xxx
```

If you wish to put the Raspberry Pi back into AP mode, you can call

```bash
sudo wifi.sh -a
```

## Create® 3 WiFi Setup

- Press both Create® 3 button 1 and 2 simultaneously until light ring turns blue

<figure class="aligncenter">
    <img src="media/create_ap_mode.jpg" alt="Create AP mode" style="width: 70%"/>
    <figcaption>Putting the Create® 3 in AP mode</figcaption>
</figure>

- The Create® 3 is now in AP mode. Connect to its WiFi network called 'Create-XXXX'
- In a browser go to 192.168.10.1
- Click connect and enter your WiFi ssid and password
- Wait for it to connect to WiFi and play a chime
- On your PC, run `ros2 topic list` to ensure that the Create® 3 is publishing its topics

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

## Updating the TurtleBot 4

It is recommended to update both the Create® 3 and the Raspberry Pi when you first use it to receive the latest bug fixes and improvements.

### Create® 3

#### Update over USB-C

Download the latest firmware from <http://edu.irobot.com/create3-latest-fw>.

Copy the firmware to the Raspberry Pi:

```bash
sudo scp ~/Downloads/Create3-G.X.Y.swu ubuntu@xxx.xxx.xxx.xxx:/home/ubuntu/
```

SSH into the Raspberry Pi and update the Create® 3 firmware over USB-C:

```bash
sudo create_update.sh Create3-G.X.Y.swu
```

or

```bash
curl -X POST --data-binary @Create3-G.X.Y.swu http://192.168.186.2/api/firmware-update
```

This may take a few minutes.

#### Update over WiFi

Find the IP address of the Create 3 on your WiFi network.

Enter the IP address into a browser. You will be taken to the Create 3 portal.

Click on update and either upload 

### Debian packages

Debian packages can be updated by calling:

```bash
sudo apt update
sudo apt install <PACKAGE>
```

For example, updating the `turtlebot4_desktop` package can be done like this:

```bash
sudo apt update
sudo apt install ros-galactic-turtlebot4-desktop
```

### Source packages

To update a source package you will need to use a terminal to manually pull changes.

For example, updating the `turtlebot4_robot` package on the `galactic` branch:

```bash
cd ~/turtlebot4_ws/src/turtlebot4_robot
git checkout galactic
git pull origin galactic
```

You will then need to rebuild the packages:

```bash
cd ~/turtlebot4_ws
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Install latest Raspberry Pi image

```warning
Installing a new image on the Raspberry Pi will delete any changes you may have made. Save your changes before proceeding.
```

If you wish to install the latest image onto your robot, follow these instructions.

The latest TurtleBot 4 Raspberry Pi images are available at http://download.ros.org/downloads/turtlebot4/.

- Download the latest image for your robot model and extract it. 
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
- Follow the instructions and wait for the SD card to be flashed. Remove the SD card from your PC.
- Ensure your Raspberry Pi 4 is not powered on before inserting the flashed SD card.
- Follow [WiFi Setup](#wifi-setup) to configure your WiFi.



