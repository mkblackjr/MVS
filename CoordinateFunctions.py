# CoordinateFunctions.py

""" 
Coordinate Manipulation and Computation Functions

This program contains functions used in the manipulation and computation of 
the cartesian and polar coordinates of the motors in the MVSS. It also contains
functions which update the motor steps to be sent as commands to the Arduino.

"""
import numpy as np
import math

###############################################################################
############################ Function Definitions #############################
###############################################################################

def xyz_to_cyl(x, y, z):
	r = math.sqrt(x**2+y**2)
	theta = math.atan2(y,x)
	theta = 2*math.pi*(theta<0)+theta
	return (r, theta, z)


def cyl_to_xyz(r, theta, z):
	x = r*math.cos(theta)
	y = r*math.sin(theta)
	return (x, y, z)


def get_current_position(position):
	R_coordinate = position[0]
	Theta_coordinate = position[1]
	Z_coordinate = position[2]
	
	return [R_coordinate, Theta_coordinate, Z_coordinate]


def calculate_steps(R_steps, T_steps, Z_steps, rR, rT, rZ):
	# Calculate difference between ideal steps and integer steps
	# and add difference to the remainders for potential incorporation
	diff_R = float(R_steps) - int(R_steps)
	diff_T = float(T_steps) - int(T_steps)
	diff_Z = float(Z_steps) - int(Z_steps)
	rR += diff_R; rT += diff_T; rZ += diff_Z

	# Assign steps to be taken as integer steps plus remainder (0 or 1)
	R_steps = int(R_steps) + int(rR)
	T_steps = int(T_steps) + int(rT)
	Z_steps = int(Z_steps) + int(rZ)

	# # If remainder incorporated into steps, subtract one from it
	rR = update_remainder(rR)
	rT = update_remainder(rT)
	rZ = update_remainder(rZ)

	return [R_steps, T_steps, -1*Z_steps, rR, rT, rZ]


def update_remainder(remainder):
	# Decrease absval of remainder by 1
	if abs(remainder) >= 1:
		remainder -= (remainder>0)-(remainder<0)
	return remainder


def encode_step_command(R, Theta, Z, wait):
	
	return bytearray('R' + str(R) + 'T' + str(Theta) + 'Z' + str(-1*Z) 
		+ 'W' + str(wait) + '\n',"ascii")


def nonMatrix_calc_dR_dTheta(x, y, z, dx, dy, dz, res):
	# in case dx, dy, dz specified rather than target coordinate
	targetX = x + dx
	targetY = y + dy
	targetZ = z + dz

	R1 = math.sqrt(x**2+y**2)
	R2 = math.sqrt(targetX**2+targetY**2)
	
	Theta1 = math.atan2(y, x)
	Theta1 = (Theta1 < 0)*2*math.pi + Theta1
	Theta2 = math.atan2(targetY, targetX)
	Theta2 = (Theta2 < 0)*2*math.pi + Theta2

	dR = R2 - R1
	print("dR = " + str(dR))
	
	# At (0,0,0) Theta = 0, but that isn't necessary for most trajectories
	# that pass through origin, so allow Theta to remain constant as the 
	# origin is approached
	if(R2 < 0.25): # 0.25 is arbitrary
		dTheta = 0
	else: 
		dTheta = Theta2 - Theta1
	print("dTheta = " + str(dTheta))

	return (dR, dTheta)

def check_path_for_x_wall(x, y, targetX, targetY):
	# Handle vertical, horizontal cases
	try:
		m = (targetY-y)/(targetX-x)
	except: 
		x_intercept = x
		m = math.inf
	try:
		# Solved system of two equations
		x_intercept = -y/m + x
	except:
		return False

	# must intercept positive x-axis AND actually cross it
	if(x_intercept > 0):
		if(abs(np.sign(targetY)-np.sign(y)) == 2):
			return True
		elif(((np.sign(targetY) == 0) & (np.sign(y) < 0)) | 
			((np.sign(y) == 0) & (np.sign(targetY) < 0))):
			return True
	else: 
		return False


def update_target(target, y, z, ideal_x, ideal_y, travel_mode, direction):
	# Only test for avoiding positive x axis if XYZ mode or directional move
	if((not travel_mode) | (direction!='G')):
		# Imaginary barrier at Theta = 0/2pi - go around it by setting waypoints
		# which simply precede the final target in the target array
		avoid_positive_x_axis = check_path_for_x_wall(ideal_x, ideal_y,
			target[0][0], target[0][1])
		if(avoid_positive_x_axis):
			if(direction=='G'):
				# 2 legs to/from (-1,0): (-1,0) -> to target
				target = np.array([[-1,0,z+(target[0][2]-z)/2],
					[target[0][0],target[0][1],target[0][2]]])
			else:
				# 3 legs at 90 angle: to y axis, to (0,y_target), to target
				target = np.array([[-1,y,z+(target[0][2]-z)/3],
					[0,target[0][1],z+2*(target[0][2]-z)/3],[target[0][0],
					target[0][1],target[0][2]]])

	return target

# # A function to calculate dR and dTheta based on a transformation matrix
# def calc_dR_dTheta(r, t, dx, dy, x, y, prev_x, prev_y,
# 	res, restart_r):
# 	# This is currently set to region of r=0.5mm, but should be updated
# 	# to reflect the viewing range of the microscope/camera so that even
# 	# subjects at the origin can be viewed (if that is possible, given
# 	# the narrow beam of the microscope)
# 	if (r >= 0.5 or restart_r):
# 		z_halt = False; restart_r = False
# 		M = np.matrix([[math.cos(t), -1*r*math.sin(t)],
# 				[math.sin(t), r*math.cos(t)]])
# 		[[dR], [dTheta]] = round(M.getI()*[[dx], [dy]],3)
# 	else:
# 		z_halt = True; restart_r = True
# 		theta_traverse = math.pi - 2*(math.pi - math.acos((x*(x-prev_x)+y*
# 			(y-prev_y))/(math.sqrt(x**2+y**2)*math.sqrt((x-prev_x)**2+
# 				(y-prev_y)**2))))
# 		dTheta = theta_traverse*res
# 		dR = 0

# 	if abs(float(dTheta)) > 2*math.pi:
# 		dTheta = np.sign(dTheta)*2*math.pi

# 	return [dR, dTheta, z_halt, restart_r]



