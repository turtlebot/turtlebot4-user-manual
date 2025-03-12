---
sort: 7
---

# **TTC-Based Speed Adjustment Using PI Control**

## **Overview**

Time-to-Collision (TTC) is a crucial metric in autonomous driving, estimating how soon a vehicle will reach an obstacle given its current speed and distance. Instead of using a **binary braking approach**, a **Proportional-Integral (PI) controller** can dynamically **adjust speed** based on how close the vehicle is to the obstacle.

Instead of **immediate braking**, this method gradually **reduces speed as the TTC decreases**, allowing for smoother deceleration and avoiding unnecessary stops.

---

## **1️⃣ Why Use PI Control for Speed Instead of Hard Braking?**

Using a **fixed speed** or a **hard stop at a certain TTC** leads to:

- 🚗 **Abrupt stops**, even when minor speed reductions would suffice.
- 🚦 **Oscillations**, where the vehicle stops and starts repeatedly.
- 🔄 **Inefficiency**, as the vehicle might brake unnecessarily.

Using a **PI controller** allows:  
✅ **Smooth deceleration** based on how fast the vehicle is approaching an obstacle.  
✅ **Adaptive speed control**, slowing down before an emergency brake is needed.  
✅ **More natural driving behavior**, reducing harsh stops.

---

## **2️⃣ PI Control Formula for Speed Adjustment**

The **error** is the difference between the desired TTC and the actual TTC:

$$
e(t) = TTC_{\text{desired}} - TTC_{\text{actual}}
$$

The **adjusted speed** is computed using the PI equation:

$$
V(t) = K_p e(t) + K_i \int e(t) dt
$$

Where:

- \(V(t)\) = Adjusted vehicle speed  
- \(K_p\) = Proportional gain (adjusts speed based on immediate TTC difference)  
- \(K_i\) = Integral gain (adjusts speed based on cumulative TTC deviation)  
- \(e(t)\) = TTC error (\(TTC_{\text{desired}} - TTC_{\text{actual}}\))  

---

## **3️⃣ Implementation Steps**

1. **Set the desired TTC threshold** (e.g., **2.0s** for smooth adjustment).  
2. **Measure the actual TTC** based on LiDAR scan and speed.  
3. **Compute the error**:  

$$
e(t) = TTC_{\text{desired}} - TTC_{\text{actual}}
$$

4. **Compute the speed adjustment** using the PI control equation.  
5. **Apply the adjusted speed** to gradually slow the vehicle.  

---

## **4️⃣ Example: PI Speed Adjustment Calculation**

Assume:

- **Desired TTC**: **2.0s**  
- **Actual TTC**: **1.2s**  
- **Proportional Gain**: \(K_p = 0.3\)  
- **Integral Gain**: \(K_i = 0.1\)  

### **Step 1: Compute the Error**
$$
e(t) = 2.0 - 1.2 = 0.8
$$

### **Step 2: Compute the Speed Adjustment**
If the **integral error** has accumulated to **1.5**:

$$
V_{\text{adjustment}} = (0.3 \times 0.8) + (0.1 \times 1.5)
$$

$$
V_{\text{adjustment}} = 0.24 + 0.15 = 0.39
$$

Thus, the **vehicle speed is reduced** by **0.39 m/s**.

---

## **5️⃣ Considerations & Tuning**

🚗 **Tuning PI Gains**:

- **If \(K_p\) is too high** → Speed reduces **too aggressively**.  
- **If \(K_p\) is too low** → Vehicle reacts **too slowly**.  
- **If \(K_i\) is too high** → Speed stays low even when TTC is safe.  
- **If \(K_i\) is too low** → Speed fluctuates too much.  

### **Tuning Approach**
1. Start with **\(K_p\) only** (Proportional control).  
2. Gradually **increase \(K_i\)** to stabilize speed adjustments.  
3. Ensure speed **never drops to zero** unless absolutely necessary.  

---

## **6️⃣ Summary**

✅ **PI-based speed adjustment prevents abrupt stops**  
✅ **Maintains safe TTC without unnecessary braking**  
✅ **Creates a smoother, more efficient driving experience**  

Would you like to add **derivative control (PID) for even better response?** 🚗💨
