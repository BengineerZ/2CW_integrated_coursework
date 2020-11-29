#!/usr/bin/python3
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

matplotlib.use("TkAgg")

N = 300
x = N * [0]
y = N * [0]
x_window = 5

fig, axis = plt.subplots()
line = axis.plot(x, y, animated=True)[0]

fft_x = []
fft_y = []
old_fft_y = []

def animate(i):
	global old_fft_y

	new_data_str = open("last_data_point.dat", "r").read()
	new_data_token = new_data_str.split()
	if len(new_data_token) == 2:
		x.append(float(new_data_token[0]))
		y.append(float(new_data_token[1]))

	fft_x = np.fft.rfftfreq(len(x[-N:-1]), d=1/60)
	fft_y = abs(np.fft.rfft(y[-N:-1]).real)

	axis = plt.axis([0, 20, 0, 100])
	avg_fft_y = [0.5*(x+y) for x,y in zip(fft_y, old_fft_y)]
	line.set_data(fft_x, avg_fft_y)

	old_fft_y = fft_y

	return line,

anim = FuncAnimation(fig, animate, interval=0, blit=True)
plt.show()
