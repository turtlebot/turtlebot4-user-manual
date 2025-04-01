---
sort: 5
---

# Hough Transforms


[![Video Thumbnail](https://img.youtube.com/vi/JmxDIuCIIcg/0.jpg)](https://www.youtube.com/watch?v=JmxDIuCIIcg "Click to Watch!")


### Introduction

The Hough (huff) Transform is a popular technique to detect any shape that can be represented in mathematical form in images, especially used for line detection. 

[OpenCV Docs Hough Line Transforms](https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html)

```python
### Step 1: Import Libraries

```python
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load image
image = cv2.imread('path_to_image.jpg')

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Get Edge
edges = cv2.Canny(gray, 50, 150, apertureSize=3)

# Apply Hough Line Transforms

lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)

Draw Detected Lines

# for a video you might want to check use:
# if lines is not None:
#   for line loop...
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

# Display Results

plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title('Detected Lines')
plt.show()

```


## Function Overview

<p align="center">
  <b>cv2.HoughLinesP(image, rho, theta, threshold, lines=None, minLineLength=None, maxLineGap=None)</b>
</p>

This function is used to detect lines in an image. It's a modification of the standard Hough Transform, called the Probabilistic Hough Transform. It's more efficient and returns the endpoints (x1, y1, x2, y2) of the detected lines.

```python
#function with parameters
lines = cv2.HoughLinesP(image, rho, theta, threshold, lines=None, minLineLength=None, maxLineGap=None)

#function in code
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
```

Parameters
1. **lines**:  
   - A vector that will store the parameters (xstart,ystart,xend,yend) of the detected lines

2. **image**: 
   - This is the output of an edge detector, typically the Canny edge detector. 
   - It should be a binary image where edges are marked with white pixels.
   - The image may be modified by the function

3. **rho** (1 in this case): 
   - The resolution of the accumulator in pixels. 
   - It represents the distance resolution in pixels. 
   - Smaller values of rho mean finer resolution. 
   - A value of 1 pixel is a good common choice.

4. **theta** (np.pi/180 in this case): 
   - Angular resolution of the accumulator in radians. 
   - It represents the angle resolution. 
   - np.pi/180 means a 1-degree resolution. 
   - The smaller the value, the finer the resolution.

5. **threshold** (100 in this case): 
   - The accumulator threshold parameter. 
   - Only those lines are returned that get enough votes (> threshold). 
   - This parameter implies the minimum length of line that will be detected. 
   - A higher threshold means fewer, but more definitive lines.

6. **minLineLength** (100 in this case): 
   - The minimum length of a line. 
   - Lines shorter than this are rejected. 
   - In this case, lines must be at least 100 pixels long to be considered valid.

7. **maxLineGap** (10 in this case): 
   - The maximum gap between segments to be treated as a single line. 
   - If the gap between parts of a line is smaller than this value, they are considered as a single line.
   - For instance, 10 means that if two segments are 10 pixels apart or less, they are considered part of the same line.

### More Details: Accumulator

#### Basic Concept
The Hough Transform is used to detect lines (or other shapes) in an image. To do this, it needs a way to identify potential lines from the edge points detected in an image (like those found using the Canny edge detector). The accumulator is essentially a data structure used for this identification process.

#### How the Accumulator Works
1. **Parameter Space Mapping**: 
   - In the case of line detection, every edge point in the image can be represented by a line equation, typically in the form y = mx + c or its polar coordinate form ρ = x cos θ + y sin θ. 
   - The accumulator maps each edge point to a parameter space (like (ρ, θ) space for lines) where each point represents a potential line.

2. **Voting System**: 
   - The accumulator operates on a voting system. 
   - For each edge point, it considers a range of possible line parameters and votes for all the lines that the edge point could belong to. 
   - In practical terms, this means incrementing the count in the accumulator cells corresponding to these lines.

3. **Identifying Strong Candidates**: 
   - The cells in the accumulator array will have varying counts based on the number of votes. 
   - A higher count in a cell means that many edge points potentially belong to the line represented by that cell's parameters. 
   - Therefore, cells with counts exceeding a certain threshold are considered as strong candidates for actual lines in the image.

#### Role in Hough Line Transform
In the Hough Line Transform, especially in the probabilistic version (cv2.HoughLinesP()):

The rho and theta parameters define the resolution of the accumulator. Smaller values mean a finer grid in the accumulator, allowing for more precise line detection but increasing computational complexity.
The threshold parameter sets the minimum number of votes a candidate line must receive to be considered a valid line.

#### Example
Consider an image with edge points. When you apply the Hough Transform:
 
- Each edge point 'votes' for all possible lines passing through it.
- These votes are tallied in the accumulator.
- Lines corresponding to cells in the accumulator with votes above the threshold are selected as the detected lines.


## Additional Resources

- [Additional OpenCV Hough Line Transform](https://docs.opencv.org/4.8.0/d6/d10/tutorial_py_houghlines.html)
- [Visualization of Hough Line Transform](https://homepages.inf.ed.ac.uk/amos/hough.html)
- [Hough Transform with OpenCV (C++/Python)](https://learnopencv.com/hough-transform-with-opencv-c-python/)
- [Line detection in python with OpenCV Houghline method](https://www.geeksforgeeks.org/line-detection-python-opencv-houghline-method/)
- [Lines detection with Hough Transform (with Video)](https://pysource.com/2018/03/07/lines-detection-with-hough-transform-opencv-3-4-with-python-3-tutorial-21/)
- [Car detection & tracking and lane detection openCV (Video Example)](https://www.youtube.com/watch?v=pQuUW3Jp8ic)
