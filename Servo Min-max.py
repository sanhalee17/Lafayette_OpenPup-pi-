from __future__ import division
import time
import Adafruit_PCA9685


pwm = Adafruit_PCA9685.PCA9685()

def set_servo_pulse(channel, pulse):
	pulse_length = 1000000
	pulse_length //=60
	print('{0}us per bit' .format (pulse_length))
	pulse *= 1000
	pulse //= pulse_length
	pwm.set_pwm(channel, 0, pulse)

pwm.set_pwm_freq(100)


while True: 


	pwm.set_pwm(15, 0, 190)
	print('0 degree')
	time.sleep(2)

	pwm.set_pwm(15,0,1050)
	print('180 degree')
	time.sleep(2)

	# pwm.set_pwm(15, 0, 252)
	# print('45 degree')
	# time.sleep(2)

	# pwm.set_pwm(15, 0, 376)
	# print('90 degree')
	# time.sleep(2)

	# pwm.set_pwm(15, 0, 500)
	# print('125 degree')
	# time.sleep(2)

	# pwm.set_pwm(15, 0, 620)
	# print('90 degree')
	# time.sleep(2)
