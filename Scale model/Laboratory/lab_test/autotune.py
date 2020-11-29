import smbus			
import time     
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal
import RPi.GPIO as GPIO
import threading

GPIO.setmode(GPIO.BOARD)

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# Stepper motor Pins and step sequence
ControlPin = [7,11,13,15]

for pin in ControlPin:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, 0)
	
seq = [ [1,0,0,0],
		[1,1,0,0],
		[0,1,0,0],
		[0,1,1,0],
		[0,0,1,0],
		[0,0,1,1],
		[0,0,0,1],
		[1,0,0,1] ]


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)
    
    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value

def reset_pendulum():
	print('Pendulum resetting, type "0" when reset...')
	for i in range(22):
		for j in range(50):
			for halfstep in range(8):
				for pin in range(4):
					GPIO.output(ControlPin[pin], seq[-halfstep][pin])
				time.sleep(0.001)
		k = input()
		if k=='0':
			return 'Pendulum reset'
		else:
			print('Continuing ...') 
	return 'Pendulum reset (no press)'
	
def set_pendulum_length(current_length, desired_length):
	corrected_steps = desired_length - current_length
	#print(corrected_steps)
	if current_length < 0 or desired_length < 0:
		print('Motion not possible: out of range - low')
		return current_length
	elif desired_length > 1100 or current_length > 1100:
		print('Motion not possible: out of range - high')
		return current_length
	for j in range(np.abs(corrected_steps)):
			for halfstep in range(8):
				for pin in range(4):
					GPIO.output(ControlPin[pin], seq[np.sign(corrected_steps)*halfstep][pin])
				time.sleep(0.001)
	return desired_length

def read_Ay_data_time(duration):
	points = []
	ts = []
	end = 0
	start = time.time()
	while(end < duration):
		
		#Read Accelerometer Y raw value
		acc_y = read_raw_data(ACCEL_YOUT_H)
		
		#Full scale range +/- 250 degree/C as per sensitivity scale factor
		Ay = acc_y/16384.0
		
		points.append(Ay)
		ts.append(time.time() - start)

		end = ts[-1]
		#print ("\tAy=%.2f g" %Ay) 	
		time.sleep(0.005)

	return (ts, points)
	
def read_plot_data():
	points = []
	ts = []
	fig,ax = plt.subplots()

	fig2,ax2 = plt.subplots()
	start = time.time()
	for i in range(800):
		
		#Read Accelerometer raw value
		
		acc_y = read_raw_data(ACCEL_YOUT_H)
		
		#Full scale range +/- 250 degree/C as per sensitivity scale factor
		Ay = acc_y/16384.0
		
		points.append(Ay)
		ts.append(time.time() - start)

		# print ("\tAy=%.2f g" %Ay) 	
		time.sleep(0.01)

	sample_rate = 800/(ts[-1])
	print(ts[-1])

	sp = np.fft.fft(points)
	freq = np.fft.fftfreq(800, 1/sample_rate)
	ax2.plot(freq, np.abs(sp.real))
	ax.plot(ts, points)
	plt.show()

def frequency_step_dynamics(frequency):
	steps = 6.8868*frequency**4 -132.68*frequency**3 + 964.72*frequency**2 -3282.8*frequency + 4717.8
	return steps
########


# Reset Pendulum to initial position:
print(reset_pendulum())
current_position = 0

# Begin autotune proceadure:
print('Press any key to continue.')
k = input()
print('Beginning auto-tuning proceadure ...')
bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
MPU_Init()
print (" Reading initial Data of Gyroscope and Accelerometer")

# Read initial accelerometer values for 3 seconds
initial_times, initial_points = read_Ay_data_time(3)

sample_rate = len(initial_times)/(initial_times[-1])
print(ts[-1])

# Perform FFT on the collected data and determine required frequency:
sp = np.fft.fft(points)
freq = np.fft.fftfreq(len(initial_times), 1/sample_rate)
indexes, _ = scipy.signal.find_peaks(np.array(np.abs(sp)), height=2, distance=int(len(initial_times)/16))
peak_frequencies = []
desired_resonance = 0
for index in indexes:
	if freq[index] > 1.0 and freq[index] < 4.0:
		peak_frequencies.append(freq[index])
		if np.array(np.abs(sp))[index] > desired_resonance:
			desired_resonance = freq[index]
		
		
print(desired_resonance)

# Move pendulum arm to desired location and record results:
set_pos = int(frequency_step_dynamics(desired_resonance))
record = threading.Thread(target=read_plot_data)
current_position = set_pendulum_length(current_position, set_pos)
record.start()





