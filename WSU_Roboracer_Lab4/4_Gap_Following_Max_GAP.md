---
sort: 4
---

# Max Gap

# Follow The Gap - Step-by-Step Breakdown

In Follow-The-Gap (FTG) obstacle avoidance, we find the safest direction for the vehicle to steer based on LiDAR data. Here’s the precise step-by-step breakdown for implementing FTG:

## Step 1: Find the Closest Obstacle and Create a Safety Bubble
- Identify the **closest LiDAR point** (minimum distance in the LiDAR scan).
- Create a **safety bubble** around this obstacle.
- Flatten (set to zero) all points inside the bubble. These points are now considered obstacles.

> **Why?** We take a pessimistic view by assuming the nearest obstacle is the biggest threat and blocking a region around it.

## Step 2: Find the Maximum Gap
- After masking the bubble, all **nonzero ranges** represent safe, free space.
- Find the **longest sequence** of consecutive nonzero points.
- This sequence is the **largest safe gap** the vehicle can steer into.

## Step 3: Pick the Best Point
- Choose the **midpoint** of the maximum gap as the best point to steer toward.
- Later, you can use more advanced methods to pick slightly to the left or right depending on the goal, but midpoint is simple and effective.

---

# Example Code for Step 1 and Step 2

```python
import numpy as np

def find_max_gap(self, free_space_ranges, bubble_radius=0.5):
    """
    Find the largest navigable gap after masking a safety bubble around closest obstacle.

    Args:
        free_space_ranges (np.array): Cleaned LiDAR ranges
        bubble_radius (float): Radius around obstacle to zero out (meters)

    Returns:
        (start_idx, end_idx): indices of the largest gap
    """
    # Step 1: Find the closest obstacle
    closest_idx = np.argmin(free_space_ranges)

    # Step 2: Create the safety bubble
    angle_increment = self.angle_increment  # Assume you store angle_increment when node starts
    num_bubble_points = int(bubble_radius / angle_increment)

    start_bubble_idx = max(0, closest_idx - num_bubble_points)
    end_bubble_idx = min(len(free_space_ranges) - 1, closest_idx + num_bubble_points)

    # Flatten the safety bubble (zero out)
    free_space_ranges[start_bubble_idx:end_bubble_idx + 1] = 0.0

    # Step 3: Find the max consecutive non-zero gap
    max_gap_size = 0
    max_start_idx = 0
    max_end_idx = 0

    current_start = None

    for i, distance in enumerate(free_space_ranges):
        if distance > 0:
            if current_start is None:
                current_start = i
        else:
            if current_start is not None:
                gap_size = i - current_start
                if gap_size > max_gap_size:
                    max_gap_size = gap_size
                    max_start_idx = current_start
                    max_end_idx = i - 1
                current_start = None

    # Handle case where gap goes to the end
    if current_start is not None:
        gap_size = len(free_space_ranges) - current_start
        if gap_size > max_gap_size:
            max_start_idx = current_start
            max_end_idx = len(free_space_ranges) - 1

    return max_start_idx, max_end_idx
```

---

# Key Points to Remember
- The car width is **12 inches = 0.3048 meters**.
- You can adjust the `bubble_radius` based on your car’s size and safety margin.
- A larger bubble will avoid tighter gaps but may limit navigation in narrow environments.

# Next Step
- After finding the maximum gap, we will implement a `find_best_point()` function to select the **steering target**.


