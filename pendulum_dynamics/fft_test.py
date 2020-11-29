import matplotlib.pyplot as plt
import numpy as np
import scipy.fftpack
from scipy.signal import butter, filtfilt
import pandas as pd
from scipy.signal import find_peaks
df = pd.read_csv ('/Users/benzandonati/Downloads/pendulum_dynamics/steps0.csv')
time = np.array(df['time'])
points = np.array(df['points'])
delta = time[-1]/(1.78*len(time))
print(time[-10:-1])
print(time[0:10])
for i in range(10):
	print((i+1)*delta)

actual_time = []
for i in range(len(time) - 1):
	actual_time.append(time[i+1] - time[i])
peaks, _ = find_peaks(actual_time, distance=10, plateau_size = 1)

# Number of sample points
N = 80

points = points[N:]
time = time[N:]
delta = time[-1]/(len(time))
# # sample spacing
# T = 1.0 / 800.0
# x = np.linspace(0.0, N*T, N)
# y = np.sin(50.0 * 2.0*np.pi*x)

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filtfilt(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y

filtered = butter_lowpass_filtfilt(points, 25 , 200)
#filtered = points
print(len(filtered))
yf = scipy.fftpack.fft(filtered)
xf = np.linspace(0.0, 1.0//(2.0*delta), N//2)
print(xf)
fig, ax = plt.subplots()
#ax.plot(xf, 2.0/N * np.abs(yf[:N//2]))
#ax.plot(time, filtered)
#ax.plot(time, points)
ax.plot(actual_time)
#ax.plot(np.array(actual_time)[peaks], "x")

peaks, _ = find_peaks(2.0/N * np.abs(yf[:N//2]), distance=3)
print(peaks)
for peak in peaks:
	print(xf[peak])
	#print(filtered[peak])
	print('.')

def frequency_step_dynamics(frequency):
	steps = 6.8868*frequency**4 -132.68*frequency**3 + 964.72*frequency**2 -3282.8*frequency + 4717.8
	return steps

print(int(frequency_step_dynamics(3.18)))

#print((len(peaks)-1)/(time[peaks[-1]] - time[peaks[0]]))
#ax.plot(xf[peaks], 2.0/N * np.abs(yf[peaks]), "x")
plt.show()