#Walking_Gait
from __future__ import division
import time
import Adafruit_PCA9685

from numpy import *
from math import *
import time
import numpy as np


# INVERSE KINEMATICS: 3-D

# robot dimensions

pwm = Adafruit_PCA9685.PCA9685()


# -----------------------
# INVERSE KINEMATICS:
# -----------------------

# robot dimensions

lf = 2.70 # femur, inches
lt = 2.60 # tibia, inches
ls = 1.40 # shoulder offset, inches

wspine = 2.00 # spine width, inches
lspine = 5.00 # spine, inches

forward = 0
turn = 1
swivel = 0
sit = 0

# establish gait parameters

gait_duration = 2 # seconds
leg_pace = 50 # pace of gait

if (forward == 1):
	x_center = 0.5
	x_stride = 1

	y_center = -1
	y_offset = 0.5

	z_center = -4
	z_lift = 1

	leg1_offset = 0			# front left
	leg2_offset = pi		# front right
	leg3_offset = pi		# back left
	leg4_offset = 0 		# back right

elif (turn == 1):
	x_center = 0
	x_stride = 0

	y_center = -1
	y_offset = 0.5

	z_center = -4
	z_lift = 1

	leg1_offset = 0			# front left
	leg2_offset = pi		# front right
	leg3_offset = pi		# back left
	leg4_offset = 0 		# back right

elif (swivel == 1):
	x_center = 0.5
	x_stride = 1

	y_center = -0.5
	y_offset = 1

	z_center = -4
	z_lift = 0

	leg1_offset = 0			# front left
	leg2_offset = 0			# front right
	leg3_offset = 0			# back left
	leg4_offset = 0 		# back right


# initialize: x, y, and z positions for each foot & femur and tibia angles for each leg
# leg indexing: 1-front left, 2-front right, 3-back left, 4-back right

t = linspace(0,gait_duration,1000)

# zeros x, y, z, angs, angf, angt

x1 = zeros(len(t))
y1 = zeros(len(t))
z1 = zeros(len(t))
angs1 = zeros(len(t))
angf1 = zeros(len(t))
angt1 = zeros(len(t))

x2 = zeros(len(t))
y2 = zeros(len(t))
z2 = zeros(len(t))
angs2 = zeros(len(t))
angf2 = zeros(len(t))
angt2 = zeros(len(t))

x3 = zeros(len(t))
y3 = zeros(len(t))
z3 = zeros(len(t))
angs3 = zeros(len(t))
angf3 = zeros(len(t))
angt3 = zeros(len(t))

x4 = zeros(len(t))
y4 = zeros(len(t))
z4 = zeros(len(t))
angs4 = zeros(len(t))
angf4 = zeros(len(t))
angt4 = zeros(len(t))


# develop functions for foot positions for given gait

for i in range(0,len(t)):
	
	if (forward == 1):
		x1[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg1_offset)
		y1[i] = y_center + y_offset*sin(leg_pace*t[i] - pi - leg1_offset)
		z1[i] = z_center + z_lift*sin(leg_pace*t[i] - leg1_offset)

		x2[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg2_offset)
		y2[i] = y_center + y_offset*sin(leg_pace*t[i] - pi - leg2_offset)
		z2[i] = z_center + z_lift*sin(leg_pace*t[i] - leg2_offset)

		x3[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg3_offset)
		y3[i] = y_center + y_offset*sin(leg_pace*t[i] - pi - leg3_offset)
		z3[i] = z_center + z_lift*sin(leg_pace*t[i] - leg3_offset)

		x4[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg4_offset)
		y4[i] = y_center + y_offset*sin(leg_pace*t[i] - pi - leg4_offset)
		z4[i] = z_center + z_lift*sin(leg_pace*t[i] - leg4_offset)

		if (z1[i]) < z_center: z1[i] = z_center
		if (z2[i]) < z_center: z2[i] = z_center
		if (z3[i]) < z_center: z3[i] = z_center
		if (z4[i]) < z_center: z4[i] = z_center

	elif (turn == 1):
		x1[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg1_offset)
		y1[i] = y_center - y_offset*sin(leg_pace*t[i] - pi - leg1_offset)
		z1[i] = z_center + z_lift*sin(leg_pace*t[i] - pi/2 - leg1_offset)

		x2[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg2_offset)
		y2[i] = y_center + y_offset*sin(leg_pace*t[i] - pi - leg2_offset)
		z2[i] = z_center + z_lift*sin(leg_pace*t[i] - pi/2 - leg2_offset)

		x3[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg3_offset)
		y3[i] = y_center + y_offset*sin(leg_pace*t[i] - pi - leg3_offset)
		z3[i] = z_center + z_lift*sin(leg_pace*t[i] - pi/2 - leg3_offset)

		x4[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg4_offset)
		y4[i] = y_center - y_offset*sin(leg_pace*t[i] - pi - leg4_offset)
		z4[i] = z_center + z_lift*sin(leg_pace*t[i] - pi/2 - leg4_offset)

		if (z1[i]) < z_center: z1[i] = z_center
		if (z2[i]) < z_center: z2[i] = z_center
		if (z3[i]) < z_center: z3[i] = z_center
		if (z4[i]) < z_center: z4[i] = z_center

	elif (swivel == 1):
		x1[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg1_offset)
		y1[i] = y_center - y_offset*sin(leg_pace*t[i] - pi - leg1_offset)
		z1[i] = z_center + z_lift*sin(leg_pace*t[i] - pi/2 - leg1_offset)

		x2[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg2_offset)
		y2[i] = y_center + y_offset*sin(leg_pace*t[i] - pi - leg2_offset)
		z2[i] = z_center + z_lift*sin(leg_pace*t[i] - pi/2 - leg2_offset)

		x3[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg3_offset)
		y3[i] = y_center + y_offset*sin(leg_pace*t[i] - leg3_offset)
		z3[i] = z_center + z_lift*sin(leg_pace*t[i] - pi/2 - leg3_offset)

		x4[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 -leg4_offset)
		y4[i] = y_center - y_offset*sin(leg_pace*t[i] - leg4_offset)
		z4[i] = z_center + z_lift*sin(leg_pace*t[i] - pi/2 - leg4_offset)

		if (z1[i]) < z_center: z1[i] = z_center
		if (z2[i]) < z_center: z2[i] = z_center
		if (z3[i]) < z_center: z3[i] = z_center
		if (z4[i]) < z_center: z4[i] = z_center


# INVERSE KINEMATICS FUNCTION

#  to solve for servo angles As (shoulder), Af (femur), and At (tibia)

def getServoAng(x, y, z, ls, lf, lt, leg):

# by geometry
	if (y<0):
		Adxy = arctan(z/y)
	else:
		Adxy = pi + arctan(z/y)

	dxy = sqrt(y**2 + z**2)
	As = Adxy - arccos(ls/dxy)

	if (leg == 1 or leg == 3):
		As = pi-As

		if (x<0):
			Ad = pi + arctan((z+ls*sin(As))/x)
		else:
			Ad = arctan((z+ls*sin(As))/x)

		d = sqrt(x**2 + (z+ls*sin(As))**2)
		Af = Ad - arccos((lf**2 + d**2 - lt**2)/(2*lf*d))
		At = pi - arccos((lf**2 + lt**2 - d**2)/(2*lf*lt))

		Af = pi-Af
		At = -At

	else:
		if (x<0):
			Ad = arctan((z+ls*sin(As))/x)
		else:
			Ad = pi + arctan((z+ls*sin(As))/x)

		d = sqrt(x**2 + (z+ls*sin(As))**2)
		Af = Ad - arccos((lf**2 + d**2 - lt**2)/(2*lf*d))
		At = pi - arccos((lf**2 + lt**2 - d**2)/(2*lf*lt))


	return As,Af,At


# --------------------------
# END OF INVERSE KINEMATICS
# --------------------------


sangf1 = zeros(len(t))
sangt1 = zeros(len(t))
sangs1 = zeros(len(t))

sangf2 = zeros(len(t))
sangt2 = zeros(len(t))
sangs2 = zeros(len(t))

sangf3 = zeros(len(t))
sangt3 = zeros(len(t))
sangs3 = zeros(len(t))

sangf4 = zeros(len(t))
sangt4 = zeros(len(t))
sangs4 = zeros(len(t))


qangf2 = zeros(len(t)) 
qangt2 = zeros(len(t)) 
qangf4 = zeros(len(t)) 
qangt4 = zeros(len(t)) 

def set_servo_pulse(channel, pulse):
	pulse_length = 1000000
	pulse_length //=60
	print('{0}us per bit' .format (pulse_length))
	pulse *= 1000
	pulse //= pulse_length
	pwm.set_pwm(channel, 0, pulse)
	pwm.set_pwm_freq(100)



#tibia and femur angles in radians
for i in range(0,len(t)):
	angs1[i], angf1[i], angt1[i] = getServoAng(x1[i], y1[i], z1[i], ls, lf, lt, 1)
	angs2[i], qangf2[i], qangt2[i] = getServoAng(x2[i], y2[i], z2[i], ls, lf, lt, 2)
	angs3[i], angf3[i], angt3[i] = getServoAng(x3[i], y3[i], z3[i], ls, lf, lt, 3)
	angs4[i], qangf4[i], qangt4[i] = getServoAng(x4[i], y4[i], z4[i], ls, lf, lt, 4)

	# angf1[i], angt1[i] = getServoAng(x1[i], z1[i], lf, lt)
	# qangf2[i], qangt2[i] = getServoAng(x2[i], z2[i], lf, lt) 
	# angf3[i], angt3[i] = getServoAng(x3[i], z3[i], lf, lt) 
	# qangf4[i], qangt4[i] = getServoAng(x4[i], y4[i], z4[i], lf, lt)

	angf2[i] = qangf2[i]
	angt2[i] = qangt2[i]
	angf4[i] = pi - qangf4[i]
	angt4[i] = pi - qangt4[i]


	print(str(angt1[i])+", "+ str(angf1[i])+", "+str(angt2[i])+", "+str(angf2[i])+", "+str(angt3[i])+", "+str(angf3[i])+", "+str(angt4[i])+", "+str(angf4[i]))


#converting the radians to servo angles
	sangf1[i]=(900*angf1[i])/(pi)
	sangt1[i]=(900*angt1[i])/(pi)
	sangs1[i]=(900*angs1[i])/(pi)
	sangf2[i]=(900*angf2[i])/(pi)
	sangt2[i]=(900*angt2[i])/(pi)
	sangs2[i]=(900*angs2[i])/(pi)
	sangf3[i]=(900*angf3[i])/(pi)
	sangt3[i]=(900*angt3[i])/(pi)
	sangs3[i]=(900*angs3[i])/(pi)
	sangf4[i]=(900*angf4[i])/(pi)
	sangt4[i]=(900*angt4[i])/(pi)
	sangs4[i]=(900*angs4[i])/(pi)

#angle offset values
tib1_offset = 900
fem1_offset = 1050
sh1_offset = 500

tib2_offset = 300
fem2_offset = 170
sh2_offset = 500

tib3_offset = 900
fem3_offset = 1050
sh3_offset = 500

tib4_offset = 300
fem4_offset = 170
sh4_offset = 500


#sending the servo angles to indivial servos 

i=0

while True:
	
	pwm.set_pwm(0, 0, tib2_offset+int(sangt2[i%len(sangt2)])) #port zero : right front tibia
	i=i+1
	i_c=i%len(sangt1)

	
	pwm.set_pwm(1, 0, fem2_offset+int(sangf2[i_c])) #port 1: right front femur
	pwm.set_pwm(2, 0, sh2_offset)#-int(sangs2[i%len(sangt2)]))               #port 2: right hip


	pwm.set_pwm(3, 0, tib1_offset-int(sangt1[i_c])) 
	pwm.set_pwm(4, 0, fem1_offset-int(sangf1[i_c])) 
	time.sleep(0.01)
		
	pwm.set_pwm(5, 0, sh1_offset+#+int(sangs1[i%len(sangt1)]))                            #port 5: left hip

	pwm.set_pwm(8, 0, fem3_offset - int(sangf3[i_c])) #port 8: left back femur
	pwm.set_pwm(9, 0, tib3_offset - int(sangt3[i_c])) #port 9: left back tibia
	pwm.set_pwm(10, 0, sh3_offset)#+int(sangs3[i%len(sangt3)]))                            #port 10: left back hip
	
	pwm.set_pwm(12, 0, fem4_offset+int(sangf4[i%len(sangf4)])) #port 6: right back femur  
	pwm.set_pwm(7, 0, tib4_offset+int(sangt4[i%len(sangt4)])) #port 7: right back tibia
	pwm.set_pwm(13, 0, sh4_offset)#-int(sangs4[i%len(sangt4)]))                           #port 11: right back hip

	print(str(time.time())+", "+str(int(sangt1[i%len(sangt1)]))+", "+ str(int(sangf1[i%len(sangf1)]))+", "+	str(int(sangt2[i%len(sangt2)])) +", "+ str(int(sangf2[i%len(sangf2)])) +", "+ str(int(sangt3[i%len(sangt3)]))+", "+ str(int(sangf3[i%len(sangf3)]))+", "+ str(int(sangt4[i%len(sangt4)]))+", "+ str(int(sangf4[i%len(sangf4)])))



