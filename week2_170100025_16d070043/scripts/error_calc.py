import numpy as np
import matplotlib.pyplot as plt

# Parameters
A = 4
a = 1
b = 2
start_angle = 0 #0 #np.pi/2
step_size = np.pi/60
write = False
if write:
	f = open("waypoints.txt", "w")

t = np.arange(start_angle, start_angle+2*np.pi, step_size) #start_angle+2*np.pi

x = A*np.cos(a*t)
y = A*np.sin(b*t)

# Error in position
x_diff = np.diff(x)
y_diff = np.diff(y)
E_pos = np.sqrt(x_diff**2 + y_diff**2)
plt.plot(t[1:], E_pos)
plt.title("Error in position")
plt.savefig("Error_pos.png")
plt.close()

# Error in angle - Version 1
E_theta = []
for iter in range(2, x.size):
	delta_y0 = y[iter-1] - y[iter-2]
	delta_y1 = y[iter] - y[iter-1]
	delta_x0 = x[iter-1] - x[iter-2]
	delta_x1 = x[iter] - x[iter-1]
	E_theta.append(np.arctan2(delta_y0*delta_x1 - delta_y1*delta_x0, delta_x0*delta_x1 + delta_y0*delta_y1))

E_theta = np.array(E_theta)*(180.0/np.pi)

plt.plot(t[2:], E_theta)
plt.ylabel("Error in degree")
plt.xlabel("t")
plt.title("Error in angle")
plt.savefig("Error_angle_version_1.png")
plt.close()

# Error in angle - Version 2
E_theta = np.arctan2(y_diff, x_diff) * (180.0/np.pi)
plt.plot(t[1:], E_theta)
plt.ylabel("Error in degree")
plt.xlabel("t")
plt.title("Error in angle")
plt.savefig("Error_angle_version_2.png")