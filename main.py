import numpy as np
import matplotlib.pyplot as plt

# Define time vector
t = np.linspace(0, 1, 1000)

# Generate square wave input signal
square_wave = np.sign(np.sin(2 * np.pi * 5 * t))

# Plot square wave signal
plt.plot(t, square_wave)
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.title('Square Wave Signal')
plt.show()
