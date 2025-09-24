import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

#Hardcoded waypoints (x, y)
waypoints = np.array([
    [x, 0.5*np.sin(0.1*x)]   # gentle sinusoidal curve
    for x in np.linspace(0, 54, 55)
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
    