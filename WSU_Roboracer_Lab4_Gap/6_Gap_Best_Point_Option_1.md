---
sort: 6
---

# 📄 Finding the Best Point: Naive Selection

After finding the largest free space (gap) in the LiDAR scan,  
the next step is to decide **where inside that gap the car should aim**.

Instead of just picking the center of the gap,  
we **improve** the method by looking for:

✅ The **farthest** point inside the gap (safest and widest direction).  
✅ The point **closest to straight ahead** (minimal unnecessary steering).

---

## 🛠 How We Find the Best Point

### Step 1: Focus on the Largest Gap
- Use only the LiDAR scan points inside the largest detected gap.
- Ignore points outside the gap.

### Step 2: Find the Farthest Point
- Identify the maximum distance inside the gap.
- This gives us the safest place to aim.

### Step 3: Break Ties by Distance to Center
- If multiple points are equally far,  
  pick the one **closest to the center** of the car's field of view (0°).
- This keeps steering smoother and avoids sharp unnecessary turns.

### Step 4: Calculate the Steering Angle
- **Important:** Because we flip the Hokuyo scan data (left is now index 0),  
  we **negate the steering angle** to drive in the correct direction.

---

## 📚 Why This Works

| Reason | Benefit |
|:-------|:--------|
| Farthest point | Safer, fewer obstacle risks |
| Closest to front | Smoother, more efficient driving |
| Negate angle | Corrects for flipped scan |

---

## ✏️ Python Code (Best Point Selection)

```python
def find_best_point(self, ranges, start_idx, end_idx):
    """
    Find the best point inside the largest gap: 
    the farthest distance, closest to straight ahead.

    Args:
        ranges (np.array): Full preprocessed LiDAR scan
        start_idx (int): Start of the gap
        end_idx (int): End of the gap

    Returns:
        best_point_idx (int): Best target index inside full scan
    """
    # Step 1: Extract the gap
    gap_ranges = ranges[start_idx:end_idx+1]

    # Step 2: Find max distance
    max_distance = np.max(gap_ranges)

    # Step 3: Find all indices with max distance
    candidates = np.where(gap_ranges == max_distance)[0]

    # Step 4: Pick the candidate closest to center of gap
    center_idx_in_gap = (len(gap_ranges) - 1) // 2
    best_candidate = min(candidates, key=lambda x: abs(x - center_idx_in_gap))

    # Step 5: Adjust back to full scan index
    best_point_idx = start_idx + best_candidate

    return best_point_idx
```

---

## ⚡ Quick Notes
- **Why flip the steering sign?**  
  The Hokuyo LiDAR is flipped (index 0 = left), so we must negate steering.
- **How much steering?**  
  Steering is proportional to how far the best point is from the center.
- **Straight driving?**  
  Happens when the best point is close to the center of the scan.

---

# 🚗 Summary
- Find largest gap ✅
- Pick farthest point inside gap ✅
- Prefer minimal steering (closest to front) ✅
- Flip steering direction because of flipped LiDAR ✅

---

# 📈 Next Steps
Once we select the best point,  
we publish a steering command to turn the car toward it!

---

# ⚠️ Weaknesses and Limitations of This Approach

While this "best point" strategy works well in simple environments,  
it has some important limitations to be aware of:

---

## 🛑 1. Tunnel Vision on a Single Gap
- The method focuses only on the largest gap.
- It ignores whether smaller gaps might actually offer safer, straighter paths.

## 🛑 2. No Prediction of Future Obstacles
- The algorithm only reacts to what it sees immediately.
- It does not plan ahead for tight spaces or dead ends.

## 🛑 3. Sensitive to Noisy LiDAR Data
- Small spikes or noise in the LiDAR scan can falsely appear as "gaps."
- Without smoothing or filtering, the car might steer erratically.

## 🛑 4. Hard Turns at High Speeds
- If the best point is far to one side, the car may need a sharp turn.
- Without adjusting speed, this can cause unsafe or unstable driving.

## 🛑 5. No Dynamic Obstacle Handling
- Moving obstacles (like people or other cars) are not specially treated.
- Static obstacle avoidance works, but moving objects could cause collisions.

---

## 📚 How More Advanced Systems Solve These Issues
- **Dynamic window approaches** consider both speed and steering feasibility.
- **Global planners** combine local gap following with a map of the world.
- **Sensor fusion** reduces noise by combining LiDAR, vision, and radar data.

---

# 🚀 Summary

| Strengths | Weaknesses |
|:---------|:-----------|
| Very simple and fast | No look-ahead |
| Good for open spaces | Sensitive to noise |
| Easy to understand | No dynamic obstacle awareness |
| Works without maps | Can steer into dead ends |
