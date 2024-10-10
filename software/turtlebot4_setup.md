---
sort: 6
---

# TurtleBot 4 Setup

The `turtlebot4_setup` repository contains scripts, configurations, and tools used for setting up and configuring the TurtleBot 4.

Source code is available [here](https://github.com/turtlebot/turtlebot4_setup).

## Install Scripts

{% tabs install %}
{% tab install galactic %}

> :warning: **ROS 2 Galactic is no longer supported** Please consider upgrading to a newer release

There are several install scripts that are used to set up the TurtleBot 4 image.

<table>
    <thead>
        <tr>
            <th>Script</th>
            <th>Usage</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>turtlebot4_setup.sh</b></td>
            <td style="white-space: nowrap;"><code>bash turtlebot4_setup.sh -m [model]</code></td>
            <td>This script should be used on a clean Ubuntu 20.04 Server image.
            It sets up installs ROS 2 Galactic as well as other dependencies of the TurtleBot 4.
            It will also install configuration files to their appropriate locations.</td>
        </tr>
        <tr>
            <td><b>galactic.sh</b></td>
            <td style="white-space: nowrap;"><code>bash galactic.sh</code></td>
            <td>Installs <i>ros-galactic-ros-base</i> as well as other useful packages. It is called from <b>turtlebot4_setup.sh</b>.</td>
        </tr>
        <tr>
            <td><b>bluetooth.sh</b></td>
            <td style="white-space: nowrap;"><code>bash bluetooth.sh</code></td>
            <td>Installs bluetooth packages.</td>
        </tr>
        <tr>
            <td><b>sd_flash.sh</b></td>
            <td style="white-space: nowrap;"><code>sudo sd_flash.sh /path/to/image</code></td>
            <td>Flashes a RPi image to a microSD card. It is used from a PC.</td>
        </tr>
        <tr>
            <td><b>install.py</b></td>
            <td style="white-space: nowrap;">
                <code>install.py [domain]</code> <i>(v0.1.2)</i> <br/>
                <code >install.py --domain [domain] --rmw [rmw]</code> <i>(v0.1.3)</i>
            </td>
            <td>Installs the robot_upstart job.</td>
        </tr>
        <tr>
            <td><b>uninstall.py</b></td>
            <td style="white-space: nowrap;">
                <code>uninstall.py</code>
            </td>
            <td>Uninstalls the robot_upstart job.</td>
        </tr>
        <tr>
            <td><b>create_update.sh</b></td>
            <td style="white-space: nowrap;">
                <code>create_update.sh /path/to/firmware.swu</code>
            </td>
            <td>Send a firmware file to the Create® 3 to update it.</td>
        </tr>
    </tbody>
</table>

```note
TurtleBot 4's come with an already set up RPi4 image, so these scripts will not be needed for most users.
```

{% endtab %}
{% tab install humble %}

There are several install scripts that are used to set up the TurtleBot 4 image.

<table>
    <thead>
        <tr>
            <th>Script</th>
            <th>Usage</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>turtlebot4_setup.sh</b></td>
            <td style="white-space: nowrap;"><code>bash turtlebot4_setup.sh -m [model]</code></td>
            <td>This script should be used on a clean Ubuntu 22.04 Server image.
            It sets up installs ROS 2 Humble as well as other dependencies of the TurtleBot 4.</td>
        </tr>
        <tr>
            <td><b>humble.sh</b></td>
            <td style="white-space: nowrap;"><code>bash humble.sh</code></td>
            <td>Installs <i>ros-humble-ros-base</i> as well as other useful packages. It is called from <b>turtlebot4_setup.sh</b>.</td>
        </tr>
        <tr>
            <td><b>bluetooth.sh</b></td>
            <td style="white-space: nowrap;"><code>bash bluetooth.sh</code></td>
            <td>Installs bluetooth packages.</td>
        </tr>
        <tr>
            <td><b>sd_flash.sh</b></td>
            <td style="white-space: nowrap;"><code>sudo sd_flash.sh /path/to/image</code></td>
            <td>Flashes a RPi image to a microSD card. It is used from a PC.</td>
        </tr>
        <tr>
            <td><b>create_update.sh</b></td>
            <td style="white-space: nowrap;">
                <code>create_update.sh /path/to/firmware.swu</code>
            </td>
            <td>Send a firmware file to the Create® 3 to update it.</td>
        </tr>
    </tbody>
</table>

```note
TurtleBot 4's come with an already set up RPi4 image, so these scripts will not be needed for most users.
```

{% endtab %}
{% endtabs %}

## Configuration Tools

{% tabs configuration %}
{% tab configuration galactic %}

> :warning: **ROS 2 Galactic is no longer supported** Please consider upgrading to a newer release

<u><b style="font-size: 20px;">Configuration Scripts</b></u>

These scripts are installed in the */usr/local/bin* folder and are used to configure the robot.

<table>
    <thead>
        <tr>
            <th>Script</th>
            <th>Usage</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>wifi.sh</b></td>
            <td style="white-space: nowrap;"><code>sudo wifi.sh -s [ssid] -p [password] -r [regulatory domain]</code></td>
            <td>Set up netplan to connect to a Wi-Fi network.</td>
        </tr>
        <tr>
            <td><b>swap_on.sh</b></td>
            <td style="white-space: nowrap;"><code>swap_on.sh</code></td>
            <td>Add 4 GB of swap memory. Useful when building packages from source.</td>
        </tr>
        <tr>
            <td><b>swap_off.sh</b></td>
            <td style="white-space: nowrap;"><code>swap_off.sh</code></td>
            <td>Remove swap memory.</td>
        </tr>
        <tr>
            <td><b>ros_config.sh</b> (v0.1.3 only)</td>
            <td style="white-space: nowrap;"><code>ros_config.sh</code></td>
            <td>Configure ROS_DOMAIN_ID and RMW.</td>
        </tr>
    </tbody>
</table>

{% endtab %}
{% tab configuration humble %}

<u><b style="font-size: 20px;">Setup Tool</b></u>

In Humble, the TurtleBot 4 has a new command line setup tool that simplifies the configuration process.
The tool can be run on the TurtleBot 4 by calling:

```bash
turtlebot4-setup
```

or

```bash
ros2 run turtlebot4_setup turtlebot4_setup
```

You will be greeted by a menu with several submenus. From here you can navigate between the menus and configure your robot.

<br/>
<u><b style="font-size: 20px;">Usage</b></u>

You can navigate up and down between the menus by using the `up` and `down` arrow keys, or `j` and `k`. To select a menu,
press `Enter`. To return or exit from a menu, you can press `q`, `Esc`, or `CTRL+C`. Some menus may only be exited with `CTRL+C`.

Menu items that indicate a variable and its value will prompt you for an input. Type in your input and press `Enter` to set the value. You may press `Enter` without any input to set the value to an empty string. You can also press `CTRL+C` to return without changing the value. Some input prompts such as `ROS_DOMAIN_ID` may check that your input is valid.

Once you have input all of your changes, you will need to select `Save` to apply the settings to your configuration files.
You can also select `Apply Defaults` to revert all values for this menu to defaults.

After you have saved all of the changes that you wanted, you can view the configuration files by selecting `View Settings`
in the main "TurtleBot4 Setup" menu. When you are happy with the changes, select `Apply Settings`. This will open a new
menu that will show you all of the changes that were saved, and prompt you for confirmation. If confirmed, the setup tool
will run various commands based on which settings were changed.

```note
Changes to the `ROS_DOMAIN_ID`, `ROBOT_NAMESPACE`, `RMW_IMPLEMENTATION` or `ROS_DISCOVERY_SERVER` environment variables will be applied to the Create® 3 as well, causing it to reboot. This includes all changes made in the Discovery Server menu.

Changes made to Wi-Fi settings will cause your SSH session to hang, and the RPi4 to reboot. This will look like the Raspberry Pi stopped communicated but is expected. You will need to close the terminal, wait for the robot to fully reboot and then initiate a new SSH connection.
```

<br/>
<u><b style="font-size: 20px;">ROS Setup</b></u>

The ROS Setup menu is used to configure the ROS environment. ROS and system configuration files are located in the `/etc/turtlebot4/` folder on the RPi4.

<table>
    <thead>
        <tr>
            <th>File</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>aliases.bash</b></td>
            <td>Contains helpful bash aliases</td>
        </tr>
        <tr>
            <td><b>cyclonedds_rpi.xml</b></td>
            <td>Default CycloneDDS configuration</td>
        </tr>
        <tr>
            <td><b>discovery.sh</b></td>
            <td>Script that starts a discovery server</td>
        </tr>
        <tr>
            <td><b>fastdds_discovery_create3.xml</b></td>
            <td>FastDDS configuration for the Create 3 in Discovery Server</td>
        </tr>
        <tr>
            <td><b>fastdds_rpi.xml</b></td>
            <td>Default FastDDS configuration</td>
        </tr>
        <tr>
            <td><b>setup.bash</b></td>
            <td>Bash file that exports environment variables to configure ROS 2.
            This file is sourced in .bashrc such that the environment is applied to all terminals</td>
        </tr>
        <tr>
            <td><b>system</b></td>
            <td>Text file with system information such as version number and TurtleBot 4 model</td>
        </tr>
    </tbody>
</table>

There are currently 3 ROS Setup submenus: Bash Setup, Discovery Server, and Robot Upstart.

<br/>
<u><b style="font-size: 18px;">Bash Setup</b></u>

The Bash Setup menu allows the user to make changes to the `/etc/turtlebot4/setup.bash` file.
This file sets environment variables that affect ROS 2.

<table>
    <thead>
        <tr>
            <th>Environment Variable</th>
            <th>Description</th>
            <th>Default</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>CYCLONEDDS_URI</b></td>
            <td>Path to CycloneDDS configuration</td>
            <td>/etc/turtlebot4/cyclonedds_rpi.xml</td>
        </tr>
        <tr>
            <td><b>FASTRTPS_DEFAULT_PROFILES_FILE</b></td>
            <td>Path to FastDDS configuration</td>
            <td>/etc/turtlebot4/fastdds_rpi.xml</td>
        </tr>
        <tr>
            <td><b>ROBOT_NAMESPACE</b></td>
            <td>Sets the robots namespace.</td>
            <td></td>
        </tr>
        <tr>
            <td><b>ROS_DOMAIN_ID</b></td>
            <td>Sets the robots domain ID. Defaults to 0.</td>
            <td>0</td>
        </tr>
        <tr>
            <td><b>RMW_IMPLEMENTATION</b></td>
            <td>Set the RMW implementation (rmw_fastrtps_cpp, rmw_cyclonedds_cpp)</td>
            <td>rmw_fastrtps_cpp</td>
        </tr>
        <tr>
            <td><b>TURTLEBOT4_DIAGNOSTICS</b></td>
            <td>Enable or disable TurtleBot 4 diagnostics</td>
            <td>1</td>
        </tr>
        <tr>
            <td><b>WORKSPACE_SETUP</b></td>
            <td>Path to the workspace setup.bash file</td>
            <td>/opt/ros/humble/setup.bash</td>
        </tr>
    </tbody>
</table>

<br/>
<u><b style="font-size: 18px;">Discovery Server</b></u>

The Discovery Server menu allows the user to enable or disable the onboard discovery server, as well as set the Server ID and Port for the onboard server. The menu also allows for selecting one additional discovery server for the robot to connect to although it is not advisable for beginners. See the [discovery server page](../setup/discovery_server.md#example-configuration) for an example on how to set up a two robot system.

<table>
    <thead>
        <tr>
            <th>Setting</th>
            <th>Description</th>
            <th>Default</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>Enabled</b></td>
            <td>Whether to use Discovery Server mode</td>
            <td>False</td>
        </tr>
        <tr>
            <td><b>Onboard Server - Port</b></td>
            <td>Discovery server port for the onboard server</td>
            <td>11811</td>
        </tr>
        <tr>
            <td><b>Onboard Server - Server ID</b></td>
            <td>Discovery server ID for the onboard server</td>
            <td>0</td>
        </tr>
        <tr>
            <td><b>Offboard Server - IP</b></td>
            <td>IP address of an optional offboard Discovery Server to connect to (disabled if set as blank)</td>
            <td><i>blank</i></td>
        </tr>
        <tr>
            <td><b>Offboard Server - Port</b></td>
            <td>Discovery server port for the offboard server</td>
            <td>11811</td>
        </tr>
        <tr>
            <td><b>Offboard Server - Server ID</b></td>
            <td>Discovery server ID for the offboard server</td>
            <td>1</td>
        </tr>
    </tbody>
</table>

```note
Enabling the discovery server will also set `RMW_IMPLEMENTATION` to `rmw_fastrtps_cpp`.

Discovery server settings are applied to the `DISCOVERY_SERVER` environment variable in `/etc/turtlebot4/setup.bash`.
```

<br/>
<u><b style="font-size: 18px;">Robot Upstart</b></u>

The robot upstart menu has menu options for interacting with the `robot_upstart` job that runs the TurtleBot 4 nodes as a background service.

<table>
    <thead>
        <tr>
            <th>Menu Option</th>
            <th>Description</th>
            <th>Bash equivalent</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>Restart</b></td>
            <td>Restart the <code>robot_upstart</code> job.</td>
            <td style="white-space: nowrap;"><code>sudo systemctl restart turtlebot4.service</code></td>
        </tr>
        <tr>
            <td><b>Start</b></td>
            <td>Start the <code>robot_upstart</code> job if it is not running.</td>
            <td style="white-space: nowrap;"><code>sudo systemctl start turtlebot4.service</code></td>
        </tr>
        <tr>
            <td><b>Stop</b></td>
            <td>Stop the <code>robot_upstart</code> job if it is running.</td>
            <td style="white-space: nowrap;"><code>sudo systemctl stop turtlebot4.service</code></td>
        </tr>
        <tr>
            <td><b>Install</b></td>
            <td>Install or reinstall the <code>robot_upstart</code> job with current ROS settings.</td>
            <td style="white-space: nowrap;"><code>install.py [model]</code></td>
        </tr>
        <tr>
            <td><b>Uninstall</b></td>
            <td>Uninstall the <code>robot_upstart</code> job. The service will no longer run on boot.</td>
            <td style="white-space: nowrap;"><code>uninstall.py</code></td>
        </tr>
    </tbody>
</table>

<br/>
<u><b style="font-size: 20px;">Wi-Fi Setup</b></u>

The Wi-Fi Setup menu allows users to easily connect the TurtleBot 4 to their Wi-Fi network, as well as configure the connection.

<table>
    <thead>
        <tr>
            <th>Menu Option</th>
            <th>Description</th>
            <th>Default</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>Wi-Fi Mode</b></td>
            <td>Set the RPi4 to act as a Wi-Fi access point or client</td>
            <td>Access Point</td>
        </tr>
        <tr>
            <td><b>SSID</b></td>
            <td>The SSID of the Wi-Fi network. <br/> In access point mode, this is the name of the network that will be broadcasted.</td>
            <td>Turtlebot4</td>
        </tr>
        <tr>
            <td><b>Password</b></td>
            <td>The password of the Wi-Fi network. This can be left empty.</td>
            <td>Turtlebot4</td>
        </tr>
        <!-- <tr>
            <td><b>Regulatory Domain</b></td>
            <td>Country Code of the country you live in.</td>
            <td>CA</td>
        </tr> -->
        <tr>
            <td><b>Band</b></td>
            <td>Which Wi-Fi band to use. Set to 'Any' if unsure.</td>
            <td>5GHz</td>
        </tr>
        <tr>
            <td><b>IP Address</b></td>
            <td>Sets a static IP address for the `wlan0` interface.</td>
            <td></td>
        </tr>
        <tr>
            <td><b>DHCP</b></td>
            <td>Whether to use DHCP to dynamically set an IP address.</td>
            <td>True</td>
        </tr>
    </tbody>
</table>

<br/>
<u><b style="font-size: 20px;">Bluetooth Setup</b></u>

This menu option launches `bluetoothctl` and allows you to pair and connect to a bluetooth device.
See [TurtleBot 4 Controller Setup](../setup/basic.md#turtlebot-4-controller-setup) for details.

<br/>
<u><b style="font-size: 20px;">View Settings</b></u>

The View Settings menu lists the TurtleBot 4 configuration files and allows you to preview them by navigating to each file.

Changes you saved in other menus will be reflected here.

<br/>
<u><b style="font-size: 20px;">Apply Settings</b></u>

Selecting "Apply Settings" will prompt the user to confirm that they want to apply these settings. It will also list all of the changes
that will be applied. When confirmed, the setup tool will run various commands based on which settings were changed.

```note
If settings that affect the Create® 3 were changed, those changes will be sent to the base over USB-C,
and the Create® 3 will then reboot to apply the settings. Changes to the Wi-Fi network will cause the tool to run
`sudo netplan apply && sudo reboot`, causing the RPi4 to update its network settings before rebooting. This will cause any SSH session to hang.
```

Once settings have been applied, you can exit the setup tool. If there were changes made to environment variables, you will
need to run `turtlebot4-source` or `source $ROBOT_SETUP` to apply them to your current terminal. Changes will be automatically
applied to any new terminals.

<br/>
<u><b style="font-size: 20px;">About</b></u>

The "About" menu displays system information and has menu options to change the TurtleBot 4 model and hostname.

```warning
Changing the model to not match the physical robot model is not recommended.
```

{% endtab %}
{% tab configuration jazzy %}

<u><b style="font-size: 20px;">Setup Tool</b></u>

In Jazzy, the TurtleBot 4 has a command line setup tool that simplifies the configuration process.
The tool can be run on the TurtleBot 4 by calling:

```bash
turtlebot4-setup
```

or

```bash
ros2 run turtlebot4_setup turtlebot4_setup
```

You will be greeted by a menu with several submenus. From here you can navigate between the menus and configure your robot.

<br/>
<u><b style="font-size: 20px;">Usage</b></u>

You can navigate up and down between the menus by using the `up` and `down` arrow keys, or `j` and `k`. To select a menu,
press `Enter`. To return or exit from a menu, you can press `q`, `Esc`, or `CTRL+C`. Some menus may only be exited with `CTRL+C`.

Menu items that indicate a variable and its value will prompt you for an input. Type in your input and press `Enter` to set the value. You may press `Enter` without any input to set the value to an empty string. You can also press `CTRL+C` to return without changing the value. Some input prompts such as `ROS_DOMAIN_ID` may check that your input is valid.

Once you have input all of your changes, you will need to select `Save` to apply the settings to your configuration files.
You can also select `Apply Defaults` to revert all values for this menu to defaults.

After you have saved all of the changes that you wanted, you can view the configuration files by selecting `View Settings`
in the main "TurtleBot4 Setup" menu. When you are happy with the changes, select `Apply Settings`. This will open a new
menu that will show you all of the changes that were saved, and prompt you for confirmation. If confirmed, the setup tool
will run various commands based on which settings were changed.

```note
Changes to the `ROS_DOMAIN_ID`, `ROBOT_NAMESPACE`, `RMW_IMPLEMENTATION` or `ROS_DISCOVERY_SERVER` environment variables will be applied to the Create® 3 as well, causing it to reboot. This includes all changes made in the Discovery Server menu.

Changes made to Wi-Fi settings will cause your SSH session to hang, and the RPi4 to reboot. This will look like the Raspberry Pi stopped communicated but is expected. You will need to close the terminal, wait for the robot to fully reboot and then initiate a new SSH connection.
```

<br/>
<u><b style="font-size: 20px;">ROS Setup</b></u>

The ROS Setup menu is used to configure the ROS environment. ROS and system configuration files are located in the `/etc/turtlebot4/` folder on the RPi4.

<table>
    <thead>
        <tr>
            <th>File</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>aliases.bash</b></td>
            <td>Contains helpful bash aliases</td>
        </tr>
        <tr>
            <td><b>cyclonedds_rpi.xml</b></td>
            <td>Default CycloneDDS configuration</td>
        </tr>
        <tr>
            <td><b>discovery.sh</b></td>
            <td>Script that starts a discovery server</td>
        </tr>
        <tr>
            <td><b>fastdds_discovery_create3.xml</b></td>
            <td>FastDDS configuration for the Create 3 in Discovery Server</td>
        </tr>
        <tr>
            <td><b>fastdds_rpi.xml</b></td>
            <td>Default FastDDS configuration</td>
        </tr>
        <tr>
            <td><b>setup.bash</b></td>
            <td>Bash file that exports environment variables to configure ROS 2.
            This file is sourced in .bashrc such that the environment is applied to all terminals</td>
        </tr>
        <tr>
            <td><b>system</b></td>
            <td>Text file with system information such as version number and TurtleBot 4 model</td>
        </tr>
    </tbody>
</table>

There are currently 3 ROS Setup submenus: Bash Setup, Discovery Server, and Robot Upstart.

<br/>
<u><b style="font-size: 18px;">Bash Setup</b></u>

The Bash Setup menu allows the user to make changes to the `/etc/turtlebot4/setup.bash` file.
This file sets environment variables that affect ROS 2.

<table>
    <thead>
        <tr>
            <th>Environment Variable</th>
            <th>Description</th>
            <th>Default</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>CYCLONEDDS_URI</b></td>
            <td>Path to CycloneDDS configuration</td>
            <td>/etc/turtlebot4/cyclonedds_rpi.xml</td>
        </tr>
        <tr>
            <td><b>FASTRTPS_DEFAULT_PROFILES_FILE</b></td>
            <td>Path to FastDDS configuration</td>
            <td>/etc/turtlebot4/fastdds_rpi.xml</td>
        </tr>
        <tr>
            <td><b>ROBOT_NAMESPACE</b></td>
            <td>Sets the robots namespace.</td>
            <td></td>
        </tr>
        <tr>
            <td><b>ROS_DOMAIN_ID</b></td>
            <td>Sets the robots domain ID. Defaults to 0.</td>
            <td>0</td>
        </tr>
        <tr>
            <td><b>RMW_IMPLEMENTATION</b></td>
            <td>Set the RMW implementation (rmw_fastrtps_cpp, rmw_cyclonedds_cpp)</td>
            <td>rmw_fastrtps_cpp</td>
        </tr>
        <tr>
            <td><b>TURTLEBOT4_DIAGNOSTICS</b></td>
            <td>Enable or disable TurtleBot 4 diagnostics</td>
            <td>1</td>
        </tr>
        <tr>
            <td><b>WORKSPACE_SETUP</b></td>
            <td>Path to the workspace setup.bash file</td>
            <td>/opt/ros/jazzy/setup.bash</td>
        </tr>
    </tbody>
</table>

<br/>
<u><b style="font-size: 18px;">Discovery Server</b></u>

The Discovery Server menu allows the user to enable or disable the onboard discovery server, as well as set the Server ID and Port for the onboard server. The menu also allows for selecting one additional discovery server for the robot to connect to although it is not advisable for beginners. See the [discovery server page](../setup/discovery_server.md#example-configuration) for an example on how to set up a two robot system.

<table>
    <thead>
        <tr>
            <th>Setting</th>
            <th>Description</th>
            <th>Default</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>Enabled</b></td>
            <td>Whether to use Discovery Server mode</td>
            <td>False</td>
        </tr>
        <tr>
            <td><b>Onboard Server - Port</b></td>
            <td>Discovery server port for the onboard server</td>
            <td>11811</td>
        </tr>
        <tr>
            <td><b>Onboard Server - Server ID</b></td>
            <td>Discovery server ID for the onboard server</td>
            <td>0</td>
        </tr>
        <tr>
            <td><b>Offboard Server - IP</b></td>
            <td>IP address of an optional offboard Discovery Server to connect to (disabled if set as blank)</td>
            <td><i>blank</i></td>
        </tr>
        <tr>
            <td><b>Offboard Server - Port</b></td>
            <td>Discovery server port for the offboard server</td>
            <td>11811</td>
        </tr>
        <tr>
            <td><b>Offboard Server - Server ID</b></td>
            <td>Discovery server ID for the offboard server</td>
            <td>1</td>
        </tr>
    </tbody>
</table>

```note
Enabling the discovery server will also set `RMW_IMPLEMENTATION` to `rmw_fastrtps_cpp`.

Discovery server settings are applied to the `DISCOVERY_SERVER` environment variable in `/etc/turtlebot4/setup.bash`.
```

<br/>
<u><b style="font-size: 18px;">Robot Upstart</b></u>

The robot upstart menu has menu options for interacting with the `robot_upstart` job that runs the TurtleBot 4 nodes as a background service.

<table>
    <thead>
        <tr>
            <th>Menu Option</th>
            <th>Description</th>
            <th>Bash equivalent</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>Restart</b></td>
            <td>Restart the <code>robot_upstart</code> job.</td>
            <td style="white-space: nowrap;"><code>sudo systemctl restart turtlebot4.service</code></td>
        </tr>
        <tr>
            <td><b>Start</b></td>
            <td>Start the <code>robot_upstart</code> job if it is not running.</td>
            <td style="white-space: nowrap;"><code>sudo systemctl start turtlebot4.service</code></td>
        </tr>
        <tr>
            <td><b>Stop</b></td>
            <td>Stop the <code>robot_upstart</code> job if it is running.</td>
            <td style="white-space: nowrap;"><code>sudo systemctl stop turtlebot4.service</code></td>
        </tr>
        <tr>
            <td><b>Install</b></td>
            <td>Install or reinstall the <code>robot_upstart</code> job with current ROS settings.</td>
            <td style="white-space: nowrap;"><code>install.py [model]</code></td>
        </tr>
        <tr>
            <td><b>Uninstall</b></td>
            <td>Uninstall the <code>robot_upstart</code> job. The service will no longer run on boot.</td>
            <td style="white-space: nowrap;"><code>uninstall.py</code></td>
        </tr>
    </tbody>
</table>

<br/>
<u><b style="font-size: 20px;">Wi-Fi Setup</b></u>

The Wi-Fi Setup menu allows users to easily connect the TurtleBot 4 to their Wi-Fi network, as well as configure the connection.

<table>
    <thead>
        <tr>
            <th>Menu Option</th>
            <th>Description</th>
            <th>Default</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>Wi-Fi Mode</b></td>
            <td>Set the RPi4 to act as a Wi-Fi access point or client</td>
            <td>Access Point</td>
        </tr>
        <tr>
            <td><b>SSID</b></td>
            <td>The SSID of the Wi-Fi network. <br/> In access point mode, this is the name of the network that will be broadcasted.</td>
            <td>Turtlebot4</td>
        </tr>
        <tr>
            <td><b>Password</b></td>
            <td>The password of the Wi-Fi network. This can be left empty.</td>
            <td>Turtlebot4</td>
        </tr>
        <!-- <tr>
            <td><b>Regulatory Domain</b></td>
            <td>Country Code of the country you live in.</td>
            <td>CA</td>
        </tr> -->
        <tr>
            <td><b>Band</b></td>
            <td>Which Wi-Fi band to use. Set to 'Any' if unsure.</td>
            <td>5GHz</td>
        </tr>
        <tr>
            <td><b>IP Address</b></td>
            <td>Sets a static IP address for the `wlan0` interface.</td>
            <td></td>
        </tr>
        <tr>
            <td><b>DHCP</b></td>
            <td>Whether to use DHCP to dynamically set an IP address.</td>
            <td>True</td>
        </tr>
    </tbody>
</table>

<br/>
<u><b style="font-size: 20px;">Bluetooth Setup</b></u>

This menu option launches `bluetoothctl` and allows you to pair and connect to a bluetooth device.
See [TurtleBot 4 Controller Setup](../setup/basic.md#turtlebot-4-controller-setup) for details.

<br/>
<u><b style="font-size: 20px;">View Settings</b></u>

The View Settings menu lists the TurtleBot 4 configuration files and allows you to preview them by navigating to each file.

Changes you saved in other menus will be reflected here.

<br/>
<u><b style="font-size: 20px;">Apply Settings</b></u>

Selecting "Apply Settings" will prompt the user to confirm that they want to apply these settings. It will also list all of the changes
that will be applied. When confirmed, the setup tool will run various commands based on which settings were changed.

```note
If settings that affect the Create® 3 were changed, those changes will be sent to the base over USB-C,
and the Create® 3 will then reboot to apply the settings. Changes to the Wi-Fi network will cause the tool to run
`sudo netplan apply && sudo reboot`, causing the RPi4 to update its network settings before rebooting. This will cause any SSH session to hang.
```

Once settings have been applied, you can exit the setup tool. If there were changes made to environment variables, you will
need to run `turtlebot4-source` or `source $ROBOT_SETUP` to apply them to your current terminal. Changes will be automatically
applied to any new terminals.

<br/>
<u><b style="font-size: 20px;">About</b></u>

The "About" menu displays system information and has menu options to change the TurtleBot 4 model and hostname.

```warning
Changing the model to not match the physical robot model is not recommended.
```

{% endtab %}
{% endtabs %}

