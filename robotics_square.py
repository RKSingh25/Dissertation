import numpy as np
import matplotlib.pyplot as plt

# Define circle radius and center position
# radius = 1.0
# center_x = 0.0
# center_y = 0.0
# L1=1
# L2=1
#
# # Define time duration and time vector
# T = 10.0  # seconds
# dt = 0.01  # seconds
# t = np.arange(0, T, dt)
# N = len(t)
#
# # Define constant angular velocity for circle
# omega = 2 * np.pi / T  # rad/s
# r=2
# # Generate x and y positions for circle
# x_desired = center_x + radius * np.cos(omega * t)
# y_desired = center_y + radius * np.sin(omega * t)
#
# x_des = r * np.cos(omega*t)
# y_des = r * np.sin(omega*t)
#
#
#
#
# fig,ax = plt.subplots()
# ax.plot(x_des,y_des)
# ax.set_xlabel('Time (s)')
# ax.set_ylabel('Control Effort 2')
# plt.show()
#
# print(len(x_des))
# print(len(t))


import numpy as np

def positive_radians(angle):
    if angle < 0:
        degrees = 360 - ((angle*(360/(2*np.pi)))*(-1))
        rad = 2*np.pi*(degrees/360)
    return rad

print(positive_radians(-4.5))