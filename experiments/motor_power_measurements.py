from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import numpy as np


# Experimental data
angular_speeds = [0, 6.2, 15.56, 23.11]
powers = [0, 256, 512, 1023]

# Here we fitting a parameters for power function into data obtained from experiments
def calculate_power(angular_speed, a, b):
    return a * angular_speed ** b

popt, pcov = curve_fit(calculate_power, angular_speeds, powers)

# Plotting the results for comparison
xdata = np.linspace(0, 24)
plt.plot(xdata, calculate_power(xdata, *popt), 'g--', label='fit: a=%5.3f, b=%5.3f' % tuple(popt))
plt.scatter(angular_speeds, powers)

plt.xlabel('x')
plt.ylabel('y')
plt.legend()

# Printing the parameters of a function
print('power = %5.2f * angular_velocity **%5.2f' % tuple(popt))