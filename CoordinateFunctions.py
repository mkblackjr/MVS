# coordTransform.py

""" 
Coordinate Transformation Program

This program receives input in either the Cartesian coordinate system (X, Y, Z)
or the cylindrical coordinate system (r, theta, z) and makes the corresponding
transformations, including the first time derivatives.

"""
import numpy as np
import math

###############################################################################
############################ Function Definitions #############################
###############################################################################

def xyz_to_cyl(x, y, z):
	r = math.sqrt(x**2+y**2)
	theta = math.atan2(y,x)
	return (r, theta, z)

def cyl_to_xyz(r, theta, z):
	x = r*math.cos(theta)
	y = r*math.sin(theta)
	return (x, y, z)

# def coordTransform(current_sys, c1, c2, c3, d1, d2, d3):

# 	M = [math.cos(theta)]

def get_current_position(position):
	
	# Initialize coordinates as strings because info is encoded as string
	R_coordinate = ""
	Theta_coordinate = ""
	Z_coordinate = ""

	# Look for R flag to specify R position
	isError = True
	for char in position:
		position = position[1:]
		if(char == 'R'):
			isError = False
			break
	if isError: # Throw error in case of mangled coordinate reading
		isError = False
		raise ValueError('The encoded position from the Arduino was not ' + \
			'formatted properly. It did not include the "R" flag.')

	# Look for T flag, everything before it is part of R coordinate
	isError = True
	for char in position:
		position = position[1:]
		if(char != 'T'):
			R_coordinate += char
		else:
			isError = False
			break
	if isError: # Throw error in case of mangled coordinate reading
		isError = False
		raise ValueError('The encoded position from the Arduino was not ' + \
			'formatted properly. It did not include the "T" flag.')

	# Look for Z flag, everything before it is part of Theta (T) coordinate
	isError = True
	for char in position:
		position = position[1:]
		if(char != 'Z'):
			Theta_coordinate += char
		else:
			isError = False
			break
	if isError: # Throw error in case of mangled coordinate reading
		isError = False
		raise ValueError('The encoded position from the Arduino was not ' + \
			'formatted properly. It did not include the "Z" flag.')

	# Add anything between Z flag and either end or subsequent R flag
	# to Z coordinate
	for char in position:
		position = position[1:]
		if(char == 'R'):
			break
		else:
			Z_coordinate += char

	# Convert the strings into floating point numbers for use in calculations
	R_coordinate = float(R_coordinate)
	Theta_coordinate = float(Theta_coordinate)
	Z_coordinate = float(Z_coordinate)

	# return the polar coordinates
	return [R_coordinate, Theta_coordinate, Z_coordinate]


def direction_move(position, direction, steps):

	# Initialize the steps to take as zero, they will be altered if necessary
	R_steps = 0; Theta_steps = 0; Z_steps = 0
	X_steps = 0; Y_steps = 0; 

	# Obtain position in Cartesian and Polar coordinates
	[R, Theta, Z] = get_current_position(position)
	[x, y, z] = cyl_to_xyz(R, Theta, Z)

	# Compute the coordinate transformation matrix
	M = np.matrix([[math.cos(Theta), -1*R*math.sin(Theta)],
		[math.sin(Theta), R*math.cos(Theta)]])

	# Determine the direction of movement and assign steps accordingly
	if direction == 'U':
		Z_steps = steps
	elif direction == 'D':
		Z_steps = -1*steps
	elif direction == 'R':
		X_steps = steps
	elif direction == 'L':
		X_steps = -1*steps
	elif direction == 'F':
		Y_steps = steps
	elif direction == 'B':
		Y_steps = -1*steps

	# try/except for case of singular matrix
	try:
		[[R_steps], [Theta_steps]] = M.getI()*[[X_steps], [Y_steps]]
	except: 
		# need a way to have system know which theta position it is in
		raise ValueError("You are at R = 0.")

	# Calculate difference between exact and integer steps for remainder calc
	diff_R = float(R_steps) - int(R_steps)
	diff_Theta = float(Theta_steps) - int(Theta_steps)
	diff_Z = float(Z_steps) - int(Z_steps)

	# return integer steps to take and exact/int differences
	return [int(R_steps), int(Theta_steps), int(Z_steps),
	diff_R, diff_Theta, diff_Z]











