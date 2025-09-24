import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

#Hardcoded waypoints (x, y)
waypoints = np.array([
    [0.0, 0.0],   [0.3, 0.6],   [0.7, 1.1],   [1.1, 1.7],   [1.5, 2.0],
    [2.0, 1.6],   [2.4, 1.0],   [2.7, 0.2],   [3.1, -0.5],  [3.6, -1.0],
    [4.0, -1.2],  [4.5, -1.3],  [5.0, -1.0],  [5.3, -0.5],  [5.7, 0.0],
    [6.0, 0.4],   [6.3, 0.8],   [6.6, 1.6],   [7.0, 1.3],   [7.5, 0.8],
    [8.0, 0.4],   [8.5, 0.0],   [8.9, -0.8],  [9.2, -1.4],  [9.6, -2.0],
    [9.8, -2.5],  [10.2, -1.7], [10.5, -1.0], [10.7, -0.2], [11.0, 0.7],
    [11.5, 1.2],  [12.0, 1.4],  [12.5, 1.1],  [12.8, 0.6],  [13.1, 0.0],
    [13.6, -0.5], [14.0, -1.1], [14.3, -1.9], [14.7, -2.2], [15.1, -1.9],
    [15.5, -1.3], [16.0, -0.6], [16.3, 0.1],  [16.6, 0.7],  [17.0, 1.4],
    [17.4, 1.7],  [17.8, 1.2],  [18.2, 0.4],  [18.6, -0.3], [19.0, -1.0]
])

#Parameter (0,1,2,...)
t = np.arange(len(waypoints))

#Create cubic splines for x(t) and y(t)
cs_x = CubicSpline(t, waypoints[:, 0])
cs_y = CubicSpline(t, waypoints[:, 1])

#Sample along the spline
t_new = np.linspace(0, len(waypoints) - 1, 200)
x_smooth = cs_x(t_new)
y_smooth = cs_y(t_new)

#Plot results
plt.figure(figsize=(6, 4))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Original waypoints')
plt.plot(x_smooth, y_smooth, 'b-', label='Smoothed path (Cubic Spline)')
plt.title("Path Smoothing Plot")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()
    