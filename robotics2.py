import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
plt.rc('font', family='Helvetica')
T = 10 # Simulation Time
t_span = (0, T)
dt = 0.001 # Time step
t = np.arange(t_span[0], t_span[1]+dt, dt)
theta1_desired = np.concatenate((np.linspace(-np.pi/2,0,int(len(t)/4)),np.linspace(0,-np.pi/4,int(len(t)/8)),np.linspace(-np.pi/4,1,int(len(t)/4)),np.linspace(1,-np.pi/8,int(3*(len(t)/8)+1))))
theta2_desired = np.concatenate((np.linspace(0,np.pi/4,int(len(t)/4)),np.linspace(np.pi/4,np.pi/8,int(len(t)/4)),np.linspace(np.pi/8,np.pi/6,int(len(t)/8)),np.linspace(np.pi/6,np.pi/3,int(3*(len(t)/8)+1))))

fig,ax = plt.subplots()
ax.plot(t,theta1_desired)
ax.set_xlabel('Time (s)', fontsize = 15)
ax.set_ylabel('Joint angle 1 (rad)', fontsize = 15)
plt.tick_params(axis='both', which='major', labelsize=16)
# ax.set_xlim(0,10.121)
plt.show()

fig2,ax = plt.subplots()
ax.plot(t,theta2_desired)
ax.set_xlabel('Time (s)', fontsize = 15)
ax.set_ylabel('Joint angle 2 (rad)', fontsize = 15)
# ax.set_xlim(0,10.121)
plt.tick_params(axis='both', which='major', labelsize=16)
plt.show()


Kp1 = np.concatenate((np.linspace(180, 420, int((len(t)/4))),np.linspace(420,180,int((len(t)/2))),np.linspace(180, 420, int((len(t)/4)+1))))
Kd1 = np.concatenate((np.linspace(24, 100, int((len(t)/4))),np.linspace(100,24,int((len(t)/2))),np.linspace(24, 100, int((len(t)/4)+1))))
Ki1 = np.concatenate((np.linspace(450, 700, int((len(t)/4))),np.linspace(700,450,int((len(t)/2))),np.linspace(450, 700, int((len(t)/4)+1))))

fig3,ax = plt.subplots()
ax.plot(t, Kp1, label = r'$K_{P_{1,2}}}$')
ax.plot(t,Kd1, label = r'$K_{D_{1,2}}$')
ax.plot(t,Ki1, label = r'$K_{I_{1,2}}$')
ax.set_xlabel('Time (s)', fontsize = 14)
ax.set_ylabel('Controller Gains', fontsize = 14)
plt.tick_params(axis='both', which='major', labelsize=14)
plt.legend(loc='upper left', fontsize = 12.5)
plt.show()
