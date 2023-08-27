import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

T = 10
t_span = (0, T)
dt = 1
t = np.arange(t_span[0], t_span[1]+dt, dt)
wind_torque = np.concatenate((1.5*np.zeros(int(2*(len(t)/3))), np.linspace(0, 14.38, int(len(t)/3)+1)))

disturbance = np.zeros((2, 1))
for i in range(len(t)):

        disturbance[0] = wind_torque[i]  # add disturbance torque to first joint
        disturbance[1] = wind_torque[i]
        print(disturbance)