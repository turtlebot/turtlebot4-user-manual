---
sort: 6
---


# **Speed Adjustment Using PID Control for TTC**

## **Overview**

In **autonomous driving and Advanced Driver Assistance Systems (ADAS)**, **Time-to-Collision (TTC)** is used to adjust vehicle speed dynamically. Instead of braking abruptly when TTC drops below a threshold, a **Proportional-Integral-Derivative (PID) controller** can **smoothly adjust speed** based on how TTC changes over time.

This approach provides **gradual deceleration and acceleration**, improving both safety and driving comfort.

---

## **1️⃣ Why Use PID Control for Speed Adjustment?**

A fixed TTC threshold (e.g., reducing speed when **TTC < 1.0s**) may cause:

- **Harsh speed reductions**, leading to inefficient driving.
- **Oscillations**, where speed fluctuates between slowing down and speeding up.
- **Overcorrections**, making the vehicle feel unstable.

A **PID controller** allows: 

✅ **Gradual speed adjustments** based on TTC deviation.  
✅ **Predictive control**, slowing down in advance if TTC is decreasing rapidly.  
✅ **Smoother driving**, eliminating unnecessary fluctuations in speed.

---

## **2️⃣ PID Control Formula for Speed Adjustment**

The **error** is defined as the difference between desired and actual TTC:

$$e(t) = TTC_{\text{desired}} - TTC_{\text{actual}}$$

The **speed adjustment** output is calculated using:

$$V_{\text{adjustment}} = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}​$$

Where:

- $V_{\text{adjustment}}$ = Speed adjustment (positive = accelerate, negative = decelerate).
- $K_{\text{p}}$ = Proportional gain (reacts to the TTC error).
- $K_{\text{i}}$ = Integral gain (corrects long-term speed errors).
- $K_{\text{d}}$ = Derivative gain (predicts sudden changes in TTC).
- $e(t)$ = TTC error.


---

## **3️⃣ How the Derivative Term Helps**

The **derivative term** predicts future risks by monitoring the **rate of change of TTC**.

**If TTC is dropping rapidly** → Reduce speed **aggressively** to prevent a collision.  
**If TTC is stable** → Hold current speed **(avoid unnecessary slowing)**.  
**If TTC is increasing** → Smoothly **increase speed** to improve efficiency.

Example:

- **Obstacle detected at a distance** → **Slowly reduce speed** (TTC decreases gradually).
- **Sudden obstacle appears** → **Rapid speed reduction** (TTC drops sharply).
- **Obstacle moves away** → **Smooth acceleration** to return to normal speed.

---

## **4️⃣ Example: PID-Based Speed Control**

### **Scenario: Adjusting Speed Based on TTC**

- **Desired TTC** = **2.0s**
- **Actual TTC** = **1.0s**
- **Proportional Gain** = $K_{\text{p}}$ = 0.5
- **Integral Gain** = $K_{\text{i}}$ = 0.1
- **Derivative Gain** = $K_{\text{d}}$ = 0.4
- **TTC is dropping at** 0.3s per second

#### **Step 1: Compute the Error**

$$e(t) = 2.0 - 1.0 = 1.0$$

#### **Step 2: Compute Speed Adjustment**

$$V_{\text{adjustment}} = (0.5 \times 1.0) + (0.1 \times \int 1.0 dt) + (0.4 \times 0.3)$$

If the accumulated **integral error** over time is **2.0**, then:

$$V_{\text{adjustment}} = (0.5 \times 1.0) + (0.1 \times 2.0) + (0.4 \times 0.3)$$

Thus, the vehicle will **reduce speed smoothly by 0.82 m/s**, instead of braking immediately.

---

## **5️⃣ Considerations & Tuning**

🛠 **Tuning Kd​ Carefully**

- **Kd​ too high** → Reacts too aggressively to minor TTC changes (unstable speed control).
- **Kd​ too low** → Delayed reaction to sudden changes (slower response).

🚗 **Tuning Strategy**

1. Start with **P-control only** to see basic responsiveness.
2. Add **I-control** to **correct steady-state speed errors**.
3. Introduce **D-control** to **anticipate sudden obstacles and prevent overcorrections**.

---

## **6️⃣ Summary**

✅ **PID-based TTC speed control prevents abrupt slowdowns**.  
✅ **The derivative term predicts risk before it happens**.  
✅ **Results in smoother, more efficient speed adjustments**.