#Sanha Lee
#Servo Angle Conversion

# Servo2.py
# Two servo motors driven by PCA9685 chip

from smbus import SMBus
from PCA9685 import PWM
import time
import numpy as np

fPWM = 50
i2c_address = 0x40 # (standard) adapt to your module
channel = np.array([0, 1, 2, 3, 4, 5, 6, 7]) # 
a = 8.5 # adapt to your servo
b = 2  # adapt to your servo

def setup():
    global pwm
    bus = SMBus(1) # Raspberry Pi revision 2
    pwm = PWM(bus, i2c_address)
    pwm.setFreq(fPWM)

def setDirection(direction):
    duty = a / 180 * direction + b
    pwm.setDuty(channel, duty)
    print "direction =", direction, "-> duty =", duty
    time.sleep(1) # allow to settle
   
print "starting"
setup()
for direction in range(0, 181, 10):
    setDirection(direction)
direction = 0    
setDirection(0)    
print "done"