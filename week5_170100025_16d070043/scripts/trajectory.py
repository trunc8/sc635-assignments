import numpy as np
import matplotlib.pyplot as plt
import sys,os

def find_distance(pos, target):
	return np.sqrt((pos[0]-target[0])**2 + (pos[1]-target[1])**2)

# Parameters
R = 4
r = 1.5
theta_0 = np.arcsin(r/(R-r))

write = True
if write:
	f = open(os.path.join(sys.path[0],"waypoints.txt"), "w")

start_pos = [0, 0]
first_diversion = [(R-r)*np.cos(theta_0), 0]

step_size = 0.5
thresh = 0.1

curr_pos = [0, 0]
while find_distance(curr_pos, first_diversion)>thresh:
	curr_pos[0] += step_size
	print("{:.3f},{:.3f}".format(curr_pos[0], curr_pos[1]))
	plt.scatter(curr_pos[0], curr_pos[1], color='blue')
	if write:
		f.write("{:.3f},{:.3f}\n".format(curr_pos[0], curr_pos[1]))

center_smaller = [(R-r)*np.cos(theta_0), (R-r)*np.sin(theta_0)]
curr_angle = -np.pi/2
end_angle = theta_0
step_size = 0.35

while end_angle - curr_angle > thresh:
	x = (R-r)*np.cos(theta_0) + r*np.cos(curr_angle)
	y = (R-r)*np.sin(theta_0) + r*np.sin(curr_angle)
	print("{:.3f},{:.3f}".format(x, y))
	curr_angle += step_size
	plt.scatter(x, y, color='black')
	if write:
		f.write("{:.3f},{:.3f}\n".format(x, y))

curr_angle = theta_0
step_size = 0.15

while curr_angle < theta_0 + 2*np.pi:
	x = R*np.cos(curr_angle)
	y = R*np.sin(curr_angle)
	print("{:.3f},{:.3f}".format(x, y))
	curr_angle += step_size
	plt.scatter(x, y, color='red')
	if write:
		f.write("{:.3f},{:.3f}\n".format(x, y))

if write:
	f.close()
plt.axis('equal')
plt.title("Offline Waypoints")
plt.savefig(os.path.join(sys.path[0],'../images/Offline_Waypoints.png'))