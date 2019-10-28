import time
import Adafruit_PCA9685

from numpy import *
from math import *
import time
import numpy as np

def set_servo_pulse(channel, pulse):
	pulse_length = 1000000
	pulse_length //=60
	print('{0}us per bit' .format (pulse_length))
	pulse *= 1000
	pulse //= pulse_length


pwm.set_pwm(channel, 0, pulse)
pwm.set_pwm_freq(60)

while True:
	pwm.set_pwm(0, 0, 100)