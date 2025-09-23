import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

#Hardcoded waypoints (x, y)
waypoints = np.array([
            [0.0, 0.0],
            [0.7, 0.3],
            [1.4, 0.8],
            [2.1, 1.3],
            [2.8, 1.1],
            [3.5, 0.7],
            [4.0, 0.0],
            [4.3, -0.6],
            [4.5, -1.2],
            [4.8, -1.8],
            [5.1, -1.5],
            [5.7, -1.0],
            [6.1, -0.3],
            [6.5, 0.5],
            [7.0, 1.2],
            [7.5, 1.8],
            [8.0, 1.5],
            [8.5, 0.7],
            [9.0, 0.0],
            [9.5, -1]
])

#Parameter (0,1,2,...)
t = np.arange(len(waypoints))

#Create cubic splines for x(t) and y(t)
cs_x = CubicSpline(t, waypoints[:, 0])
cs_y = CubicSpline(t, waypoints[:, 1])

#Sample along the spline
t_new = np.linspace(0, len(waypoints) - 1, 100)
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
    