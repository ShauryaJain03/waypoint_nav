import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Hardcoded waypoints (x, y)
waypoints = np.array([
    [0.0, 0.0],    # Start
    [1.0, 1.0],    # Diagonal upward
    [2.5, -0.5],   # Sharp turn downward
    [3.0, 2.0],    # Steep climb back up
    [4.5, 1.5],    # Quick small bend
    [5.0, 3.0],    # Sudden upward shift
    [6.5, 0.0],    # Long diagonal descent
    [7.0, -1.5],   # Further down (tests stability in sharp drops)
    [8.0, 1.0],    # Rapid climb
    [9.0, 0.0],    # Return to baseline
])

# Parameter (0,1,2,...)
t = np.arange(len(waypoints))

# Create cubic splines for x(t) and y(t)
cs_x = CubicSpline(t, waypoints[:, 0])
cs_y = CubicSpline(t, waypoints[:, 1])

# Sample along the spline
t_new = np.linspace(0, len(waypoints) - 1, 100)
x_smooth = cs_x(t_new)
y_smooth = cs_y(t_new)

# Plot results
plt.figure(figsize=(6, 4))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Original waypoints')
plt.plot(x_smooth, y_smooth, 'b-', label='Smoothed path (Cubic Spline)')
plt.title("Path Smoothing Debug Plot")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()
    