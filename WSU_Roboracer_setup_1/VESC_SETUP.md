---
sort: 1
---

# WSU Roboracer: Configuring the VESC

## 🚨 Important Safety Tips
```warning
 - Put your car on an elevated stand so that its wheels can turn without it going anywhere. If you don’t have an RC car stand, you can use the box that came with your Jetson.
 - Make sure you hold on to the car while testing the motor to prevent it from flying off the stand.
 - Make sure there are no objects (or people) in the vicinity of the wheels while testing.
 - Use a fully charged LiPO battery instead of a power supply to ensure the motor has enough current to spin up.
```

### **Equipment Required:**
- Fully built RoboRacer vehicle
- Box or [Car stand](https://www.amazon.com/Duratrax-Tech-Deluxe-Truck-Stand/dp/B0014T74MS) to put the vehicle on
- Laptop/computer (does not need to be running Linux)

### **Approximate Time Investment:** 1 hour

```note
If using the VESC mkIV (e.g., hardware based on VESC 4.12), see [this repository](https://github.com/f1tenth/vesc_firmware) for details on building firmware for corresponding VESCs and prebuilt firmware for different VESC hardware versions.
```
---

## 1. Installing the VESC Tool
We need to configure the VESC so that it works with our motor and vehicle transmission. Before you start, you'll need to install the [VESC Tool](https://vesc-project.com/vesc_tool). You'll have to register for an account to download it. Add the free tier tool to your cart (you don't have to fill in any information other than your email). After checkout, a download link will be sent to your email. The software is available for Linux, Windows, and macOS.

---

## 2. Powering the VESC
First, we need to power the VESC. Plug the battery in and make sure the polarity is correct. Note that you don't need to turn on the Powerboard for configuring the VESC.

![Powering the VESC](media/vesc01.JPG)

Next, unplug the USB cable of the VESC from the Jetson NX and plug the USB into your laptop that's running the VESC Tool. You may want to use a longer cable.

![Connecting the VESC](media/vesc02.JPG)

---

## 3. Connecting the VESC to Your Laptop
Launch the VESC Tool. On the Welcome page, press the **AutoConnect** button at the bottom left of the page. After the VESC is connected, you should see an updated status at the bottom right of the screen.

![AutoConnect in VESC Tool](media/connect.png)

---

## 4. Updating the Firmware on the VESC
We are currently using an older firmware version of the VESC. Download it [here](https://drive.google.com/file/d/19veWRe745p3efOyn-Ff3RRYlADhp_c5V/view?usp=sharing). This assumes that you are also using version 4.12 of the VESC hardware. Read more about it [here](https://github.com/RacecarJ/vesc-firmware/tree/master/firmware).

Switch to the **Custom File** tab and upload the file that you downloaded. There will be a warning; continue to upload.

With VESC Tool versions released after Mar. 31, 2021, you can use the latest default firmware. To enable servo out, go to:

**App Settings > General > Enable Servo Output** in the VESC Tool.

![Enable Servo Output](media/servo.png)

Before VESC Tool version 2.05, you can enable servo out using a non-default firmware. Click on the **Firmware** tab on the left. Then, check **Show non-default firmwares** at the bottom left. Select **VESC_servoout.bin** from the right-side options. Press the **down arrow button** at the bottom right to update the firmware.

![Update Firmware](media/firmware.png)

---

## 5. Uploading the Motor Configuration XML
After updating the firmware, select **Load Motor Configuration XML** from the dropdown menu and upload the provided XML file from [here](https://drive.google.com/file/d/1-KiAh3hCROPZAPeOJtXWvfxKY35lhhTO/view?usp=sharing). Click on the **Write Motor Configuration** button (down arrow with letter "M") to apply the settings.

![Upload Motor Config XML](media/xml.png)

---

## 6. Detecting and Calculating Motor Parameters
To detect and calculate the FOC motor parameters, navigate to the **FOC** tab under **Motor Settings**. At the bottom of the screen, click the four buttons in sequence, following the on-screen prompts. 

> ⚠ **Warning:** The motor will make noise and spin during the measurement process. Ensure the wheels are clear of obstacles.

![Detecting Motor Parameters](media/detect_motor.png)

After the parameters are measured, the fields at the bottom of the screen should turn green. Click **Apply**, then **Write Motor Configuration**.

![Apply Motor Parameters](media/apply_motor.png)

---

## 7. Changing the Openloop Hysteresis and Openloop Time
Go to the **Sensorless** tab and change the **Openloop Hysteresis** and **Openloop Time** to **0.01**. Click **Write Motor Configuration**.

![Change Openloop Time](media/open_loop.png)

---

## 8. Tuning the PID Controller
To monitor the RPM response, navigate to **Realtime Data > Data Analysis** and click **Stream Realtime Data (RT button)**. Then, go to the **RPM** tab.

![Stream Realtime Data](media/realtime.png)

To create a step response:
- Set a target RPM (2000 - 10000 RPM).
- Click **Play** to start the motor.
- Click **STOP** to stop.

> ⚠ **Ensure the vehicle's wheels are free from obstacles.**

![Motor Step Response](media/response.png)

To fine-tune the speed PID controller, navigate to **PID Controllers > Motor Settings** and adjust the gains. If oscillations occur, modify the **Speed PID Kd Filter**.

![Adjust PID Gains](media/pid_gains.png)

---

## 9. Changing the Hardware Speed Limit
By default, the motor configuration sets a safe max RPM. To change the limit, go to:

**Motor Settings > General**, then modify the **Max ERPM** for forward and backward rotations.

![Change Max ERPM](media/erpm.png)

> 🚨 **Warning:** See the **Odometry Tuning** section in the software stack setup for converting vehicle velocity to ERPM to calculate a safe max ERPM.

---

