
# Walking Gait - 3D

from __future__ import division
import time
import Adafruit_PCA9685

from numpy import *
from math import *
import time
import numpy as np

pwm = Adafruit_PCA9685.PCA9685()
zero = 0

# -----------------------
# INVERSE KINEMATICS: 3-D
# -----------------------

# robot dimensions

lf = 2.70 # femur, inches
lt = 2.60 # tibia, inches
ls = 1.40 # shoulder offset, inches

wspine = 2.00 # spine width, inches
lspine = 5.00 # spine, inches


# ACTION CHOICES: forward, turn, swivel, sideways, jump

action = "jump"

# -------------------------
# establish gait parameters
# -------------------------

gait_duration = 2 # seconds

if (action == "forward"):
	leg_pace = 80 # pace of gait

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

elif (action == "turn"):
	leg_pace = 80 # pace of gait

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

elif (action == "swivel"):
	leg_pace = 80 # pace of gait

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

elif (action == "sideways"):
	leg_pace = 200 # pace of gait

	x_center = -0.1
	x_stride = 0

	y_center = -1
	y_offset = 0.5

	z_center = -4.75
	z_lift = 0.75

	leg1_offset = 0			# front left
	leg2_offset = pi		# front right
	leg3_offset = pi/2			# back left
	leg4_offset = 3*pi/2 		# back right

elif (action == "jump"):
	leg_pace = 100 # pace of gait

	x_center_front = 0.5
	x_center_back = -0.5
	x_stride = 0

	y_center = -1
	y_offset = 0

	z_center = -3
	z_lift = -2.2

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

	if (action == "forward"):
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

	elif (action == "turn"):
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

	elif (action == "swivel"):
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

	elif (action == "sideways"):
		x1[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg1_offset)
		y1[i] = y_center + y_offset*sin(leg_pace*t[i] - pi - leg1_offset)
		z1[i] = z_center + z_lift*sin(leg_pace*t[i] - pi/2 - leg1_offset)

		x2[i] = x_center + x_stride*sin(leg_pace*t[i] - pi/2 - leg2_offset)
		y2[i] = y_center - y_offset*sin(leg_pace*t[i] - pi - leg2_offset)
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

	elif (action == "jump"):
		x1[i] = x_center_front
		y1[i] = y_center

		x2[i] = x_center_front
		y2[i] = y_center

		x3[i] = x_center_back
		y3[i] = y_center

		x4[i] = x_center_back
		y4[i] = y_center
		
		if (i<250):
			z1[i] = z_center
			z2[i] = z_center
			z3[i] = z_center
			z4[i] = z_center
		else:
			z1[i] = z_center + z_lift
			z2[i] = z_center + z_lift
			z3[i] = z_center + z_lift
			z4[i] = z_center + z_lift


# ---------------------------
# INVERSE KINEMATICS FUNCTION
# ---------------------------

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
	angs2[i], angf2[i], angt2[i] = getServoAng(x2[i], y2[i], z2[i], ls, lf, lt, 2)
	angs3[i], angf3[i], angt3[i] = getServoAng(x3[i], y3[i], z3[i], ls, lf, lt, 3)
	angs4[i], angf4[i], angt4[i] = getServoAng(x4[i], y4[i], z4[i], ls, lf, lt, 4)

	if angf1[i] > 0: angf1[i] = angf1[i] - 2*pi
	if angf3[i] > 0: angf3[i] = angf3[i] - 2*pi

	print(str(angt1[i])+", "+ str(angf1[i])+", "+str(angt2[i])+", "+str(angf2[i])+", "+str(angt3[i])+", "+str(angf3[i])+", "+str(angt4[i])+", "+str(angf4[i]))


#converting the radians to servo angles

	sangf1[i]=(860*angf1[i])/(pi/2)
	sangt1[i]=(800*angt1[i])/(pi/2)
	sangs1[i]=(760*angs1[i])/(pi/2)

	sangf2[i]=(760*angf2[i])/(pi/2)
	sangt2[i]=(760*angt2[i])/(pi/2)
	sangs2[i]=(760*angs2[i])/(pi/2)

	sangf3[i]=(800*angf3[i])/(pi/2)
	sangt3[i]=(760*angt3[i])/(pi/2)
	sangs3[i]=(760*angs3[i])/(pi/2)

	sangf4[i]=(760*angf4[i])/(pi/2)
	sangt4[i]=(760*angt4[i])/(pi/2)
	sangs4[i]=(760*angs4[i])/(pi/2)


#angle offset values
tib1_offset = 1800
fem1_offset = 1800
sh1_offset = 2000

tib2_offset = 300
fem2_offset = 400
sh2_offset = 500

tib3_offset = 1800
fem3_offset = 1800
sh3_offset = 2000

tib4_offset = 300
fem4_offset = 400
sh4_offset = 500


#sending the servo angles to indivial servos

i=0

while True:

	i = i+1
	i_c = i%len(sangt1)

	# check zeros

	if (zero == 1):
		pwm.set_pwm(0, 0, tib2_offset+760)
		pwm.set_pwm(1, 0, fem2_offset+760)
		pwm.set_pwm(2, 0, sh2_offset)

		pwm.set_pwm(3, 0, tib1_offset-800)
		pwm.set_pwm(4, 0, fem1_offset-860)
		pwm.set_pwm(5, 0, sh1_offset)

		time.sleep(0.01)

		pwm.set_pwm(8, 0, fem3_offset-800)
		pwm.set_pwm(9, 0, tib3_offset-760)
 		pwm.set_pwm(10, 0, sh3_offset)

		pwm.set_pwm(12, 0, fem4_offset+760)
		pwm.set_pwm(7, 0, tib4_offset+760)
		pwm.set_pwm(13, 0, sh4_offset)


	# control servos

	else:
		pwm.set_pwm(0, 0, tib2_offset + int(sangt2[i_c]))		#port 0: right front tibia
		pwm.set_pwm(1, 0, fem2_offset + int(sangf2[i_c]))		#port 1: right front femur
		pwm.set_pwm(2, 0, sh2_offset - int(sangs2[i_c]))		#port 2: right front hip

		pwm.set_pwm(3, 0, tib1_offset + int(sangt1[i_c]))		#port 3: left front tibia
		pwm.set_pwm(4, 0, fem1_offset + int(sangf1[i_c]))		#port 4: left front femur
		pwm.set_pwm(5, 0, sh1_offset - int(sangs1[i_c]))		#port 5: left front hip

		time.sleep(0.01)

		pwm.set_pwm(8, 0, fem3_offset + int(sangf3[i_c]))		#port 8: left back femur
		pwm.set_pwm(9, 0, tib3_offset + int(sangt3[i_c]))		#port 9: left back tibia
		pwm.set_pwm(10, 0, sh3_offset - int(sangs3[i_c]))		#port 10: left back hip


		pwm.set_pwm(12, 0, fem4_offset + int(sangf4[i_c]))		#port 6: right back femur
		pwm.set_pwm(7, 0, tib4_offset + int(sangt4[i_c]))		#port 7: right back tibia
		pwm.set_pwm(13, 0, sh4_offset - int(sangs4[i_c]))		#port 11: right back hip


	print(str(time.time())+", "+str(int(sangt1[i%len(sangt1)]))+", "+ str(int(sangf1[i%len(sangf1)]))+", "+ str(int(sangs1[i%len(sangs1)])))






