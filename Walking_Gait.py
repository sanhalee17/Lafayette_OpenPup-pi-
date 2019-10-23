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
z_lift = 0.7

leg1_offset = pi/4		# front left
leg2_offset = 5*pi/4	# front right
leg3_offset = 0			# back left
leg4_offset = pi 		# back right


# initialize: x and z positions for each foot & femur and tibia angles for each leg
# leg indexing: 1-front left, 2-front right, 3-back left, 4-back right

t = linspace(0,gait_duration,1000)

x1 = zeros(len(t))
z1 = zeros(len(t))
zf1 = zeros(len(t))
angf1 = zeros(len(t))
angt1 = zeros(len(t))

x2 = zeros(len(t))
z2 = zeros(len(t))
zf2 = zeros(len(t))
angf2 = zeros(len(t))
angt2 = zeros(len(t))

x3 = zeros(len(t))
z3 = zeros(len(t))
zf3 = zeros(len(t))
angf3 = zeros(len(t))
angt3 = zeros(len(t))

x4 = zeros(len(t))
z4 = zeros(len(t))
zf4 = zeros(len(t))
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

qangf2 = zeros(len(t)) 
qangt2 = zeros(len(t)) 
qangf4 = zeros(len(t)) 
qangt4 = zeros(len(t)) 



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
	pwm.set_pwm_freq(100)

#setting the servos to home position
# pwm.set_pwm(0, 0, 500)
# pwm.set_pwm(1, 0, 500)
# pwm.set_pwm(2, 0, 500)
# pwm.set_pwm(3, 0, 500)
# pwm.set_pwm(4, 0, 500)
# pwm.set_pwm(5, 0, 500)
# pwm.set_pwm(6, 0, 500)
# pwm.set_pwm(7, 0, 500)
# pwm.set_pwm(8, 0, 500)
# pwm.set_pwm(9, 0, 500)
# pwm.set_pwm(10, 0, 500)
# pwm.set_pwm(11, 0, 500)



#calculating 2D IK angles

for i in range(0,len(t)):
	zf1[i] = z_lift*sin(leg_pace*t[i] - leg1_offset-pi/2)
	
	x1[i] = x_center + x_stride*sin(leg_pace*t[i]  - leg1_offset)
	if zf1[i]>=0:
	   z1[i] = z_center + z_lift*sin(leg_pace*t[i] - leg1_offset-pi/2)
	else:
	   z1[i] = z_center
	#z1[i] = z_center + z_lift*sin(leg_pace*t[i] - leg1_offset)
	#print(x1[i])

	zf2[i] = z_lift*sin(leg_pace*t[i] - leg2_offset-pi/2)
	x2[i] = x_center + x_stride*sin(leg_pace*t[i]  - leg2_offset)
	if zf2[i]>=0:
		z2[i] = z_center + z_lift*sin(leg_pace*t[i] - leg2_offset-pi/2)
	else:
		z2[i] = z_center

	zf3[i] = z_lift*sin(leg_pace*t[i] - leg3_offset-pi/2)
	x3[i] = x_center + x_stride*sin(leg_pace*t[i]  - leg3_offset)
	if zf3[i]>=0:
		z3[i] = z_center + z_lift*sin(leg_pace*t[i] - leg3_offset-pi/2)
	else:
		z3[i] = z_center

	zf4[i] = z_lift*sin(leg_pace*t[i] - leg4_offset-pi/2)
	
	x4[i] = x_center + x_stride*sin(leg_pace*t[i]  - leg4_offset)
	if zf4[i] >=0:
		z4[i] = z_center + z_lift*sin(leg_pace*t[i] - leg4_offset)
	else:
		z4[i] = z_center


#tibia and femur angles in radians
	angf1[i], angt1[i] = getServoAng(x1[i], z1[i], lf, lt)
	qangf2[i], qangt2[i] = getServoAng(x2[i], z2[i], lf, lt) 
	angf3[i], angt3[i] = getServoAng(x3[i], z3[i], lf, lt) 
	qangf4[i], qangt4[i] = getServoAng(x4[i], z4[i], lf, lt)

	angf2[i] = 2 * pi - qangf2[i]
	angt2[i] = 2 * pi - qangt2[i]
	angf4[i] = 2 * pi - qangf4[i]
	angt4[i] = 2 * pi - qangt4[i]



#converting the radians to servo angles
	sangf1[i]=(900*angf1[i])/(2*pi)
	print(sangf1[i])
	sangt1[i]=(900*angt1[i])/(2*pi)
	sangf2[i]=(900*angf2[i])/(2*pi)
	sangt2[i]=(900*angt2[i])/(2*pi)
	sangf3[i]=(900*angf3[i])/(2*pi)
	sangt3[i]=(900*angt3[i])/(2*pi)
	sangf4[i]=(900*angf4[i])/(2*pi)
	sangt4[i]=(900*angt4[i])/(2*pi)


#sending the servo angles to indivial servos 

i=0
while True:
	pwm.set_pwm(0, 0, int(sangt2[i%len(sangt2)])-48) #port zero : right front tibia
	i=i+1
	
	pwm.set_pwm(1, 0, int(sangf2[i%len(sangf2)])-200) #port 1: right front femur
	pwm.set_pwm(2, 0, 500)                            #port 2: right hip


	pwm.set_pwm(3, 0, (610-241)+int(sangt1[i%len(sangt1)])) 
	pwm.set_pwm(4, 0, (610-89) +int(sangf1[1])+int(sangf1[i%len(sangf1)])) 
	
	pwm.set_pwm(5, 0, 500)                            #port 5: left hip

	pwm.set_pwm(8, 0, (610-89)+int(sangf3[i%len(sangf3)])) #port 8: left back femur
	pwm.set_pwm(9, 0, (610-241)+int(sangt3[i%len(sangt3)])) #port 9: left back tibia
	pwm.set_pwm(10, 0, 500)                            #port 10: left back hip
	
	#pwm.set_pwm(6, 0, int(sangf4[i%len(sangf4)])-200) #port 6: right back femur  
	pwm.set_pwm(7, 0, int(sangt4[i%len(sangt4)])-48) #port 7: right back tibia
	pwm.set_pwm(11, 0, 500)                           #port 11: right back hip

	
	# time.sleep(2)
	# pwm.set_pwm(7, 0, sangf3[i])
	# pwm.set_pwm(8, 0, sangt3[i])
	# print('back left leg')

	# time.sleep(2)
	# pwm.set_pwm(10, 0, sangf4[i])
	# pwm.set_pwm(11, 0, sangt4[i])a

	# print(sangf1[i])
# # 	#This vector is defined as 

# # # function to solve for servo angles Af (femur) and At (tibia)






	






