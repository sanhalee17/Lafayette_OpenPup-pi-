#Walking_Gait
from __future__ import division
import time
import Adafruit_PCA9685

from numpy import *
from math import *
import time
import numpy as np

# INVERSE KINEMATICS: 2-D

# robot dimensions

pwm = Adafruit_PCA9685.PCA9685()

lf = 2.70 # femur, inches
lt = 2.60 # tibia, inches
ls = 5.00 # spine, inches


# establish gait parameters

gait_duration = 2 # seconds
leg_pace = 25 # pace of gait

x_center = -0.5
x_stride = 1.5

z_center = -3.5
z_lift = 0.5

leg1_offset = pi/4		# front left
leg2_offset = 5*pi/4	# front right
leg3_offset = 0			# back left
leg4_offset = pi 		# back right


# initialize: x and z positions for each foot & femur and tibia angles for each leg
# leg indexing: 1-front left, 2-front right, 3-back left, 4-back right

t = linspace(0,gait_duration,1000)

x1 = zeros(len(t))
z1 = zeros(len(t))
angf1 = zeros(len(t))
angt1 = zeros(len(t))

x2 = zeros(len(t))
z2 = zeros(len(t))
angf2 = zeros(len(t))
angt2 = zeros(len(t))

x3 = zeros(len(t))
z3 = zeros(len(t))
angf3 = zeros(len(t))
angt3 = zeros(len(t))

x4 = zeros(len(t))
z4 = zeros(len(t))
angf4 = zeros(len(t))
angt4 = zeros(len(t))

sangf1 = zeros(len(t))
sangt1 = zeros(len(t))

sangf2 = zeros(len(t))
sangt2 = zeros(len(t))

sangf3 = zeros(len(t))
sangt3 = zeros(len(t))

sangf4 = zeros(len(t))
sangt4 = zeros(len(t))



# develop functions for foot positions for given gait



def getServoAng(x, z, lf, lt):
	if (x<0):
		Ad = arctan(z/x)
	else:
		Ad = pi + arctan(z/x)

	d = sqrt(x**2 + z**2)

	Af = Ad - arccos((lf**2 + d**2 - lt**2)/(2*lf*d))
	At = pi - arccos((lf**2 + lt**2 - d**2)/(2*lf*lt))

	

	return Af,At
def set_servo_pulse(channel, pulse):
	pulse_length = 1000000
	pulse_length //=60
	print('{0}us per bit' .format (pulse_length))
	pulse *= 1000
	pulse //= pulse_length
	pwm.set_pwm(channel, 0, pulse)

pwm.set_pwm_freq(60)


for i in range(0,len(t)):
	x1[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg1_offset)
	z1[i] = z_center + z_lift*sin(leg_pace*t[i] - leg1_offset)
	#print(x1[i])
	x2[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg2_offset)
	z2[i] = z_center + z_lift*sin(leg_pace*t[i] - leg2_offset)

	x3[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg3_offset)
	z3[i] = z_center + z_lift*sin(leg_pace*t[i] - leg3_offset)

	x4[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg4_offset)
	z4[i] = z_center + z_lift*sin(leg_pace*t[i] - leg4_offset)

	angf1[i], angt1[i] = getServoAng(x1[i], z1[i], lf, lt)
	angf2[i], angt2[i] = getServoAng(x2[i], z2[i], lf, lt) 
	angf3[i], angt3[i] = getServoAng(x3[i], z3[i], lf, lt) 
	angf4[i], angt4[i] = getServoAng(x4[i], z4[i], lf, lt)

	sangf1[i]=(1000*angf1[i])/(2*pi)
	sangt1[i]=int((1000*angt1[i])/(2*pi))
	sangf2[i]=int((1000*angf2[i])/(2*pi))
	sangt2[i]=int((1000*angt2[i])/(2*pi))
	sangf3[i]=int((1000*angf3[i])/(2*pi))
	sangt3[i]=int((1000*angt3[i])/(2*pi))
	sangf4[i]=int((1000*angf4[i])/(2*pi))
	sangt4[i]=int((1000*angt4[i])/(2*pi))


i=0
while True:
	pwm.set_pwm(0, 0, int(sangf1[i%len(sangf1)]))
	i=i+1
	time.sleep(0.05)
# 	pwm.set_pwm(6, 0, 376)
# 	pwm.set_pwm(9, 0, 376)
# 	pwm.set_pwm(0, 0, 376)

# 	pwm.set_pwm(1, 0, int(sangf1[i])) 
# # 	pwm.set_pwm(2, 0, sangt1[i])
# # 	print('front left leg')

# # 	time.sleep(2)
# # 	pwm.set_pwm(4,0,sangf2[i])
# # 	pwm.set_pwm(5,0,sangt2[i])
# # 	print('front right leg')

# # 	time.sleep(2)
# # 	pwm.set_pwm(7, 0, sangf3[i])
# # 	pwm.set_pwm(8, 0, sangt3[i])
# # 	print('back left leg')

# # 	time.sleep(2)
# # 	pwm.set_pwm(10, 0, sangf4[i])
# # 	pwm.set_pwm(11, 0, sangt4[i])

# # 	print(sangf1[i])
# # 	#This vector is defined as 

# # # function to solve for servo angles Af (femur) and At (tibia)






	






