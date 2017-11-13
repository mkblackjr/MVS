import tkinter as tk
from tkinter import *
from PIL import ImageTk, Image
import math
import serial
import CoordinateFunctions as cf
import numpy as np
import time
import random

###############################################################################
############################## Class Definition ###############################
###############################################################################

class motorsUI(tk.Tk):
	
	# Initializer
	def __init__(self, serial_connection, scale_factor, step_increment,
		trajectory_res, minimum_distance, ideal_color, step_color):
		
		tk.Tk.__init__(self)

		# # This could really use some cleaning up # #

		# Frame
		self.frame = tk.Frame(self, width=400, height=400)
		self.frame.pack(side="top", fill="both", expand=True)
		
		# Exit Button
		self.b_exit = tk.Button(self.frame, text="Exit", command=lambda: 
			self.exit_program(serial_connection))
		self.b_exit.grid(row=6, column=1)

		# Frames for directional buttons
		self.f_up = tk.Frame(self.frame, height=100, width=100) 
		self.f_up.grid(row=0,column=1)
		self.f_down = tk.Frame(self.frame, height=100, width=100)
		self.f_down.grid(row=2,column=1)
		self.f_right = tk.Frame(self.frame, height=100, width=100)
		self.f_right.grid(row=1,column=2)
		self.f_left = tk.Frame(self.frame, height=100, width=100)
		self.f_left.grid(row=1,column=0)
		self.f_forwardbackward = tk.Frame(self.frame, height=100, width=100)
		self.f_forwardbackward.grid(row=1,column=1)

		# Directional buttons
		self.b_up = tk.Button(self.f_up, text="Up", command=lambda: 
			self.move_direction(serial_connection, 'U', scale_factor, 
			step_increment, ideal_color, step_color))
		self.b_down = tk.Button(self.f_down, text="Down", command=lambda: 
			self.move_direction(serial_connection, 'D', scale_factor, 
			step_increment, ideal_color, step_color))
		self.b_right = tk.Button(self.f_right, text="Right", command=lambda: 
			self.move_direction(serial_connection, 'R', scale_factor, 
			step_increment, ideal_color, step_color))
		self.b_left = tk.Button(self.f_left, text="Left", command=lambda: 
			self.move_direction(serial_connection, 'L', scale_factor, 
			step_increment, ideal_color, step_color))
		self.b_forward = tk.Button(self.f_forwardbackward, text="Forward", 
			command=lambda: self.move_direction(serial_connection, 'F', 
			scale_factor, step_increment, ideal_color, step_color))
		self.b_backward = tk.Button(self.f_forwardbackward, text="Backward", 
			command=lambda: self.move_direction(serial_connection, 'B', 
			scale_factor, step_increment, ideal_color, step_color))
		self.b_up.pack() 
		self.b_down.pack() 
		self.b_right.pack() 
		self.b_left.pack() 
		self.b_forward.grid(row=0,column=0) 
		self.b_backward.grid(row=0,column=1)

		# Slider for step input
		self.step_slider = tk.Scale(self.frame, from_=0, to=3, length=600, 
			tickinterval=0.1 ,orient=HORIZONTAL)
		self.step_slider.grid(row=3, columnspan=3)

		# Frame for X, Y, Z target coordinates
		self.XYZ_frame = tk.Frame(self.frame, height=100, width=100) 
		self.XYZ_frame.grid(row=4, columnspan=3)
		
		# Button for targeted movement execution
		self.go_button = tk.Button(self.XYZ_frame, text="Go", command=lambda: 
			self.go(serial_connection, minimum_distance, 
			ideal_color, step_color))
		self.go_button.grid(row=2, column=1)

		# X Target Coordinate Entry Box
		self.X_labelText = StringVar()
		self.X_labelText.set("Target X Coordinate (in mm)")
		self.X_labelDir = Label(self.XYZ_frame, 
			textvariable=self.X_labelText, height=1)
		self.X_labelDir.grid(row=0,column=0)
		self.X_directory = StringVar(None)
		self.X_dirname = Entry(self.XYZ_frame, 
			textvariable=self.X_directory, width=20)
		self.X_dirname.grid(row=1,column=0)
		
		# Y Target Coordinate Entry Box
		self.Y_labelText = StringVar()
		self.Y_labelText.set("Target Y Coordinate (in mm)")
		self.Y_labelDir = Label(self.XYZ_frame, 
			textvariable=self.Y_labelText, height=1)
		self.Y_labelDir.grid(row=0,column=1)
		self.Y_directory = StringVar(None)
		self.Y_dirname = Entry(self.XYZ_frame, 
			textvariable=self.Y_directory, width=20)
		self.Y_dirname.grid(row=1,column=1)
		
		# Z Target Coordinate Entry Box
		self.Z_labelText = StringVar()
		self.Z_labelText.set("Target Z Coordinate (in mm)")
		self.Z_labelDir = Label(self.XYZ_frame, 
			textvariable=self.Z_labelText, height=1)
		self.Z_labelDir.grid(row=0,column=2)
		self.Z_directory = StringVar(None)
		self.Z_dirname = Entry(self.XYZ_frame, 
			textvariable=self.Z_directory, width=20)
		self.Z_dirname.grid(row=1,column=2)

		# Position mapping settings
		self.white = "#ffffff"
		self.blank_rect_width = 10
		self.grid_path = "/Users/mac/Documents/MVSS/CircularGrid.gif"
		# self.position = serial_connection.readline().decode("utf-8")
		self.position = "R0T0Z0" # Initially at origin
		steps_per_rev_motor = 4096
		rads_per_step_motor = 2*math.pi/steps_per_rev_motor
		self.MM_PER_STEP_DZ = 0.0062 # Number of milimeters per step for DR
		self.RADS_PER_STEP_DT = 0.00147 # Number of rads per step for DTheta
		self.MM_PER_STEP_DR = 0.123 # Number of milimeters per step for DZ
		self.resolution = trajectory_res

		# Position map (drawn points on canvas with polar grid background)
		self.grid_PIL = Image.open(self.grid_path)
		self.grid_image = ImageTk.PhotoImage(self.grid_PIL)
		self.canvas = tk.Canvas(self.frame, width=self.grid_PIL.width, 
			height=self.grid_PIL.height)
		self.canvas.grid(row=5, columnspan=3)
		self.canvas.update_idletasks()
		self.canvas.create_image((0,0), image=self.grid_image, anchor=NW)
		
		# The following rectangles eliminate the image's built-in scale
		self.canvas.create_rectangle(0,0,self.blank_rect_width,
			self.canvas.winfo_height(),	outline=self.white, fill=self.white)
		self.canvas.create_rectangle(0,
			self.canvas.winfo_height()-self.blank_rect_width-5,
			self.canvas.winfo_width(),self.canvas.winfo_height(), 
			outline=self.white, fill=self.white)
		self.canvas.create_rectangle(
			self.canvas.winfo_width()-self.blank_rect_width-7,
			0,self.canvas.winfo_width(),self.canvas.winfo_height(), 
			outline=self.white, fill=self.white)
		self.canvas.create_rectangle(0,0,self.canvas.winfo_width(),
			self.blank_rect_width,outline=self.white, fill=self.white)

		# # # Other features to eventually add # # #

		# Slider for speed
		# Point and click target setting


	def move_direction(self, open_serial_port, direction, scale, increment,
		ideal_color, step_color):
		# Initializations
		remainder_R = 0; remainder_Theta = 0; remainder_Z = 0
		clear_canvas = True

		# Acquire scale factor from slider
		# This will eventually contain the desired movement in physical space
		steps = self.step_slider.get()*scale/self.MM_PER_STEP_DR
		
		# Debugging - remainder check
		R = 0; Theta = 0; Z = 0
		
		if steps != 0:
			while abs(steps) > 0:

				self.wait_for_arduino(open_serial_port)

				# Acquire integer steps and their remainders
				[R_steps, Theta_steps, Z_steps, diff_R, diff_Theta,
				diff_Z] = cf.direction_move(self.position,direction,increment)

				[R_steps, Theta_steps, Z_steps, remainder_R, remainder_Theta,
				remainder_Z] = self.calculate_steps(R_steps, Theta_steps,
					Z_steps, remainder_R, remainder_Theta, remainder_Z) 
				
				# Obtain new Polar coord location for map drawing
				R = R + R_steps*self.MM_PER_STEP_DR
				Theta = Theta + Theta_steps*self.RADS_PER_STEP_DT
				Z = Z + Z_steps*self.MM_PER_STEP_DZ

				# Obtain new xyz coords (ideal and stepped) and update map
				[x, y, z] = cf.cyl_to_xyz(R, Theta, Z)
				clear_canvas = self.update_canvas([x, y, z], [x, y, z], 
					ideal_color, step_color, clear_canvas)
				
				# Write step command to serial port (arduino)
				toWrite = self.encode_step_command(R_steps, Theta_steps, Z_steps)
				print(toWrite)
				open_serial_port.write(toWrite)

				steps -= increment*((steps > 0) - (steps < 0))

		# if scale was zero, no steps will be computed
		else: print("No movement was requested.\n")


	def go(self, open_serial_port, d_threshold,
		ideal_color, step_color):
		# Initializations
		remainder_R = 0; remainder_Theta = 0; remainder_Z = 0
		target_reached = False
		clear_canvas = True

		# Acquire target position from entry boxes
		[x_target, y_target, z_target] = self.acquire_target_pos()

		# Acquire current x, y, z position from encoded string
		[R, Theta, Z] = cf.get_current_position(self.position)
		[x, y, z] = cf.cyl_to_xyz(R, Theta, Z)
		ideal_x = x; ideal_y = y; ideal_z = z

		iterations = 0

		# Continue to compute dR, dTheta, dZ until target is "reached"
		while not target_reached:
			
			self.wait_for_arduino(open_serial_port)

			iterations += 1
			print("This is iteration " + str(iterations))

			print("x = " + str(x) + " y = " + str(y) + " z = " + str(z))
			
			# Calculate next leg of trajectory in physical space
			dx = x_target - x; dy = y_target - y; dz = z_target - z
			
			# Check if target has been reached
			d = math.sqrt(dx**2 + dy**2 + dz**2)
			if (d < d_threshold):
				target_reached = True
				# Notify user when target has been reached
				print("\nTarget reached. Input another command, or " + \
					"press 'Exit' to close the window.\n")
				open_serial_port.write(b"R0T0Z0\n")
				break

			if self.resolution == 0:
				target_reached = True
				print("\nA terminal point was reached based on the " + \
					"specified self.resolution")
				open_serial_port.write(b"R0T0Z0\n")
				break
			
			# Compute dR, dTheta using coordinate transformation matrix
			[dR, dTheta] = self.compute_dR_dTheta(R, Theta, dx, dy)

			# Calculate the ideal number of steps suggested by coord transform
			R_steps = float(dR)/self.MM_PER_STEP_DR/(self.resolution)
			Theta_steps = float(dTheta)/self.RADS_PER_STEP_DT/(self.resolution)
			Z_steps = float(dz)/self.MM_PER_STEP_DZ/(self.resolution)
			[R_steps, Theta_steps, Z_steps, remainder_R, remainder_Theta,
			remainder_Z] = self.calculate_steps(R_steps, Theta_steps, Z_steps,
			remainder_R, remainder_Theta, remainder_Z)
			
			# Write computed integer steps to serial port in encoded form
			# 'R___T___Z___'
			toWrite = self.encode_step_command(R_steps, Theta_steps, Z_steps)
			print(toWrite)
			open_serial_port.write(toWrite)

			# Compute ideal movement to check accuracy 
			# of trajectory following algorithm
			ideal_dx = (x_target - ideal_x) / (self.resolution)
			ideal_dy = (y_target - ideal_y) / (self.resolution)
			ideal_dz = (z_target - ideal_z) / (self.resolution)
			ideal_x = round(ideal_x + ideal_dx,3)
			ideal_y = round(ideal_y + ideal_dy,3)
			ideal_z = round(ideal_z + ideal_dz,3)
			[R_ideal, Theta_ideal, Z_ideal] = cf.xyz_to_cyl(ideal_x,
				ideal_y, ideal_z)

			print("The ideal trajectory is currently at (" + str(ideal_x) + \
				", " + str(ideal_y) + ", " + str(ideal_z) + ")\n")

			# Compute actual movement based on steps taken
			R_check = R + R_steps*self.MM_PER_STEP_DR
			Theta_check = Theta + Theta_steps*self.RADS_PER_STEP_DT
			Z_check = Z + Z_steps*self.MM_PER_STEP_DZ
			[check_x, check_y, check_z] = cf.cyl_to_xyz(R_check,
				Theta_check, Z_check)

			self.update_canvas([ideal_x, ideal_y, ideal_z], [check_x,
				check_y, check_z], ideal_color, step_color, clear_canvas)
			clear_canvas = False

			# Update position to reflect steps taken and encode it properly
			self.position = "R" + str(R_check) + "T" + str(Theta_check) + \
			"Z" + str(Z_check) # line for debugging when not plugged into arduino

			# Acquire x, y, z position from encoded string for next loop
			[R, Theta, Z] = cf.get_current_position(self.position)
			[x, y, z] = cf.cyl_to_xyz(R, Theta, Z)

			# Comment this out for infinte horizon gradient descent method
			self.resolution -= 1

			# time.sleep(0.01)


	def update_canvas(self, ideal_pos, step_pos, ideal_color,
		step_color, clear_canvas):
		# Apply scale factor to transform pixels to mm
		scale_factor = 10

		# Clear the previous trajectory from the map when starting a new path
		# and redraw the map (with built-in scale eliminated)
		if clear_canvas:
			self.canvas.delete("all")
			self.canvas.create_image((0,0),image=self.grid_image,anchor=NW)
			self.canvas.create_rectangle(0,0,self.blank_rect_width,
				self.canvas.winfo_height(),outline=self.white,fill=self.white)
			self.canvas.create_rectangle(0,self.canvas.winfo_height()
				-self.blank_rect_width-5,self.canvas.winfo_width(),
				self.canvas.winfo_height(),outline=self.white,fill=self.white)
			self.canvas.create_rectangle(self.canvas.winfo_width()
				-self.blank_rect_width-7,0,self.canvas.winfo_width(),
				self.canvas.winfo_height(),outline=self.white,fill=self.white)
			self.canvas.create_rectangle(0,0,self.canvas.winfo_width(),
				self.blank_rect_width,outline=self.white,fill=self.white)

		# Create new point by moving according to dx, dy, dz
		new_x = scale_factor*ideal_pos[0]
		new_y = scale_factor*ideal_pos[1]
		new_z = scale_factor*ideal_pos[2]
		self.canvas.create_oval(self.canvas.winfo_width()/2-1+new_x,
			self.canvas.winfo_height()/2-1-new_y,self.canvas.winfo_width()/2
			+1+new_x,self.canvas.winfo_height()/2+1-new_y,outline=ideal_color,
			fill=ideal_color)

		# Create new point by moving according to step_x, step_y, step_z
		new_step_x = scale_factor*step_pos[0]
		new_step_y = scale_factor*step_pos[1]
		new_step_z = scale_factor*step_pos[2]
		self.canvas.create_oval(self.canvas.winfo_width()/2-1+new_step_x,
			self.canvas.winfo_height()/2-1-new_step_y,self.canvas.winfo_width()
			/2+1+new_step_x,self.canvas.winfo_height()/2+1-new_step_y,
			outline=step_color,fill=step_color)
		
		# Refresh canvas
		self.canvas.update()

		return False


	def wait_for_arduino(self,serial_port):
		# Continue reading from serial port until arduino is ready
		ready = serial_port.readline().decode("utf-8")
		while(ready != "Go\n"):
			ready = serial_port.readline().decode("utf-8")
			print("waiting\n")
			# time.sleep(1)


	def exit_program(self,serial_port):
		serial_port.write(b"QUIT")
		time.sleep(1)
		self.destroy()


	def update_remainder(self,remainder):
		if abs(remainder) > 1:
			remainder -= (remainder>0)-(remainder<0)

		return remainder


	def encode_step_command(self,R, Theta, Z):
		
		return bytearray('R' + str(R) + 'T' + str(Theta) + 'Z' + str(-1*Z) 
			+ '\n',"ascii")


	def acquire_target_pos(self):
		# Handle the empty entry case (empty = 0)
		try: 
			x = float(self.X_dirname.get())
		except:
			if self.X_dirname.get() == "":
				x = 0
			else:
				self.X_dirname.delete(0,END)
				raise ValueError('Target X Coordinate must be a real number.')
		try: 
			y = float(self.Y_dirname.get())
		except:
			if self.Y_dirname.get() == "":
				y = 0
			else:
				self.Y_dirname.delete(0,END)
				raise ValueError('Target Y Coordinate must be a real number.')
		try: 
			z = float(self.Z_dirname.get())
		except:
			if self.Z_dirname.get() == "":
				z = 0
			else:
				self.Z_dirname.delete(0,END)
				raise ValueError('Target Z Coordinate must be a real number.')

		# Print selected target in mm to the terminal for user validation
		print("Target has the following coordinates (X, Y, Z): (" +
			str(x) + ", " + str(y) + ", " + str(z) + ")")

		return [x, y, z]

	def compute_dR_dTheta(self,r, t, dx, dy):
		M = np.matrix([[math.cos(t), -1*r*math.sin(t)],
				[math.sin(t), r*math.cos(t)]])
		try:
			[[dR], [dTheta]] = M.getI()*[[dx], [dy]]
		except:
			# If matrix is singular, use random dR, dTheta to escape origin
			dR = 5*random.randint(0,1); dTheta = 5*random.randint(0,1)

		return [dR, dTheta]

	def calculate_steps(self,R_steps, T_steps, Z_steps, rR, rT, rZ):

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
		rR = self.update_remainder(rR)
		rT = self.update_remainder(rT)
		rZ = self.update_remainder(rZ)

		return [R_steps, T_steps, Z_steps, rR, rT, rZ]

