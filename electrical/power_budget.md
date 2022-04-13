---
sort: 4
---

# Power Budget

The total power made available by the Create 3 output power adapter is 28.8W. This supplies the USB-C connector mated to Raspberry Pi, and the two pin auxiliary VBAT connector combined. Since the two connectors share this power amongst them, a rise in consumption of one will lead to reduction of available power for the other. Thus, although maximum theoretical power consumption of individual components is mentioned in Table 2, the true limiting factor is the remaining power available to the whole system.

<figure>
    <figcaption style="text-align:center;">Table 1: Nominal power consumption</figcaption>
    <table class="center-table">
        <thead>
            <tr>
                <th>Source</th>
                <th>TurtleBot 4 Lite (W)</th>
                <th>TurtleBot 4 (W)</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td>Raspberry Pi 4B</td>
                <td>4</td>
                <td>4</td>
            </tr>
            <tr>
                <td>OAK-D-Pro</td>
                <td>-</td>
                <td>5</td>
            </tr>
            <tr>
                <td>OAK-D-Lite</td>
                <td>3.5</td>
                <td>-</td>
            </tr>
            <tr>
                <td>RPLIDAR A1M8</td>
                <td>2.3</td>
                <td>2.3</td>
            </tr>
            <tr>
                <td>Fan</td>
                <td>0.8</td>
                <td>1.3</td>
            </tr>
            <tr>
                <td>UI Board</td>
                <td>-</td>
                <td>2.4</td>
            </tr>
            <tr>
                <td>User Power</td>
                <td>-</td>
                <td>*Limited By Remaining Power</td>
            </tr>
            <tr>
                <td>USB-C Ports</td>
                <td>-</td>
                <td>*Limited By Remaining Power</td>
            </tr>
            <tr>
                <td></td>
                <td></td>
                <td></td>
            </tr>
            <tr>
                <td><b>Total Power Draw</b></td>
                <td><b>10.6</b></td>
                <td><b>15</b></td>
            </tr>
            <tr>
                <td><b>Total Available Power</b></td>
                <td><b>28.8</b></td>
                <td><b>28.8</b></td>
            </tr>
            <tr>
                <td><b>*Remaining Power</b></td>
                <td><b>18.2</b></td>
                <td><b>13.8</b></td>
            </tr>
        </tbody>
    </table>
</figure>

<figure>
    <figcaption style="text-align:center;">Table 2: Maximum power consumption of individual components and systems</figcaption>
    <table class="center-table">
        <thead>
            <tr>
                <th>Source</th>
                <th>Operating Voltage (V)</th>
                <th>Max current draw (A)</th>
                <th>Max Power (W)</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td>User Interface Board</td>
                <td colspan=2></td>
                <td>~40</td>
            </tr>
            <tr>
                <td>4 USB-C ports</td>
                <td>5</td>
                <td>3</td>
                <td>15</td>
            </tr>
            <tr>
                <td>USB Hub Controller</td>
                <td>1.2, 3.3</td>
                <td>1.3</td>
                <td>1.8</td>
            </tr>
            <tr>
                <td>OLED Display</td>
                <td>12</td>
                <td>0.031</td>
                <td>0.372</td>
            </tr>
            <tr>
                <td>User LEDs</td>
                <td>5</td>
                <td>0.007</td>
                <td>0.17</td>
            </tr>
            <tr>
                <td rowspan=4>USER PWR Ports</td>
                <td>VBAT</td>
                <td>0.3</td>
                <td>4.32</td>
            </tr>
            <tr>
                <td>12</td>
                <td>0.3</td>
                <td>3.6</td>
            </tr>
            <tr>
                <td>5</td>
                <td>0.5</td>
                <td>2.5</td>
            </tr>
            <tr>
                <td>3.3</td>
                <td>0.25</td>
                <td>0.825</td>
            </tr>
            <tr>
                <td>OAK-D-Lite</td>
                <td>5</td>
                <td>1</td>
                <td>5</td>
            </tr>
            <tr>
                <td>OAK-D-Pro (connected to RPi)</td>
                <td>5</td>
                <td>1</td>
                <td>5</td>
            </tr>
            <tr>
                <td>OAK-D-Pro (connected to UI Board)</td>
                <td>5</td>
                <td>1.5</td>
                <td>7.5</td>
            </tr>
            <tr>
                <td>RPLIDAR A1M8</td>
                <td>5</td>
                <td>0.6</td>
                <td>3</td>
            </tr>
            <tr>
                <td>Blower Fan</td>
                <td>5</td>
                <td>0.25</td>
                <td>1.25</td>
            </tr>
            <tr>
                <td>Axial Fan</td>
                <td>5</td>
                <td>0.15</td>
                <td>0.75</td>
            </tr>
            <tr>
                <td>Raspberry Pi 4B</td>
                <td>5</td>
                <td>1.2</td>
                <td>6</td>
            </tr>
        </tbody>
    </table>
</figure>

```note
Not accounting for inefficiencies of components, and power loss. Assuming 
USB hub speed operating with all ports at SuperSpeed, and OLED is set to max 
brightness.
```
