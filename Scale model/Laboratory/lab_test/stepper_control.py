import RPi.GPIO as GPIO
import time
import numpy as np

GPIO.setmode(GPIO.BOARD)

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

k = 0
'''
for i in range(0):
	for halfstep in range(8):
		for pin in range(4):
			GPIO.output(ControlPin[pin], seq[-halfstep][pin])
		time.sleep(0.001)
'''

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

print(reset_pendulum())
pos = 0
for i in range(5):
	
	set_pos = int(input())
	#print(pos)
	
	pos = set_pendulum_length(pos, set_pos)

GPIO.cleanup()





#def move_pendulum(steps):
	### Note: This is an absolute move
