---
sort: 3
---

# FAQ

## Common issues with the Create® 3

## Common issues with the Raspberry Pi 4B

### 1. Waiting to connect to bluetoothd...

This issue is usually a result of the bluetooth service being stopped.

To start the service again, run `sudo systemctl start bluetooth`.


### 2. No default controller available

This error occurs if you are attempting to connect a bluetooth device to the Raspberry Pi with `sudo bluetoothctl` and the `hciuart` service throws errors.

To fix this, call `sudo systemctl disable hciuart` and then reboot the Pi with `sudo reboot`.

Once the Pi has restarted, call `sudo systemctl restart hciuart`. Now you can run `sudo bluetoothctl` again and the bluetooth controller should be found.

## Common issues with the user PC

### 1. Create® 3 topics are not visible

First, check that the Create® 3 is connected to your WiFi network. You should be able to access the Create® 3 portal by entering the Create® 3 IP address in a browser. For information on how to connect the Create® 3 to WiFi, check the [quick start guide](../overview/quick_start.md#create%C2%AE-3-wifi-setup).

If it is connected to WiFi, check if you can see Create® 3 topics on the Raspberry Pi.

If topics are visible on the Raspberry Pi, ensure that your PC has the following configuration set for CycloneDDS:

```xml
<CycloneDDS>
    <Domain>
        <General>
            <DontRoute>true</DontRoute>
        </General>
    </Domain>
</CycloneDDS> 
```

To set this configuration automatically, add the following line to your `~/.bashrc` file.

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><DontRoute>true</></></></>'
```

If topics are not visible on the Raspberry Pi, you may need to restart the Create® 3 application through the portal, or reboot the robot.

