import numpy as np
import matplotlib.pyplot as plt

# Parameters
A = 4
a = 1
b = 2
start_angle = np.pi/2
step_size = 0.1
write = True
if write:
	f = open("waypoints.txt", "w")

t = np.arange(start_angle, start_angle+2*np.pi, step_size)

x = A*np.cos(a*t)
y = A*np.sin(b*t)

for iter in range(x.size):
	plt.scatter(x[iter], y[iter], color='blue')
	print("{:.3f}, {:.3f}".format(x[iter], y[iter]))
	if write:
		f.write("{:.3f}, {:.3f}\n".format(x[iter], y[iter]))

if write:
	f.close()
plt.show()