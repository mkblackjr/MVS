import tkinter as tk
from tkinter import *
from PIL import ImageTk, Image
import math
import CoordinateFunctions as cf
import SerialFunctions as sf
import numpy as np
import time

###############################################################################
############################## Class Definition ###############################
###############################################################################

class motorsUI(tk.Tk):
	
	# Initializer
	def __init__(self, arduino_filler, scale_factor, step_increment,
		trajectory_res, minimum_distance, ideal_color, step_color):
		
		tk.Tk.__init__(self)

		# # This could really use some cleaning up # #

		# # Class variables
		self.travel_mode = 0
		# Position mapping settings
		self.white = "#ffffff"
		self.blank_rect_width = 10
		self.grid_path = "/Users/mac/Documents/MVSS/CircularGrid.gif"
		self.position = [0,0,0]# "R0T0Z0" # Initially at origin
		self.ideal_position = self.position
		# Physical parameters
		in_to_mm = 25.4
		steps_per_rev_motor = 4096
		rads_per_step_motor = 2*math.pi/steps_per_rev_motor
		gear_diameter = 20 # mm
		planetary_gear_diameter = 5.25*in_to_mm # mm
		gear_ratio = gear_diameter/planetary_gear_diameter
		motor_step_correction_factor = 501/4096
		# self.MM_PER_STEP_DZ = 0.0062 # Number of milimeters per step for DR
		# self.RADS_PER_STEP_DT = 0.00147 # Number of rads per step for DTheta
		# self.MM_PER_STEP_DR = 0.123 # Number of milimeters per step for DZ
		self.MM_PER_STEP_DZ = 0.0062*motor_step_correction_factor
		self.RADS_PER_STEP_DT = rads_per_step_motor*gear_ratio
		self.MM_PER_STEP_DR = gear_diameter*math.pi/steps_per_rev_motor
		self.execution_time_per_step = 0.0010664 # seconds per step

		# Frame
		self.frame = tk.Frame(self, width=400, height=400)
		self.frame.pack(side="top", fill="both", expand=True)
		
		# Exit Button
		self.b_exit = tk.Button(self.frame, text="Exit",
			command=lambda: self.exit_program())
		self.b_exit.grid(row=8, column=1)

		# Frames for directional buttons
		self.f_up = tk.Frame(self.frame, height=100, width=100) 
		self.f_up.grid(row=1,column=1)
		self.f_down = tk.Frame(self.frame, height=100, width=100)
		self.f_down.grid(row=3,column=1)
		self.f_right = tk.Frame(self.frame, height=100, width=100)
		self.f_right.grid(row=2,column=2)
		self.f_left = tk.Frame(self.frame, height=100, width=100)
		self.f_left.grid(row=2,column=0)
		self.f_forwardbackward = tk.Frame(self.frame, height=100, width=100)
		self.f_forwardbackward.grid(row=2,column=1)

		# Directional buttons
		self.b_up = tk.Button(self.f_up, text="Up", command=lambda: 
			self.move('U', trajectory_res,
				minimum_distance, ideal_color, step_color))
		self.b_down = tk.Button(self.f_down, text="Down", command=lambda: 
			self.move('D', trajectory_res,
				minimum_distance, ideal_color, step_color))
		self.b_right = tk.Button(self.f_right, text="Right", command=lambda: 
			self.move('R', trajectory_res,
				minimum_distance, ideal_color, step_color))
		self.b_left = tk.Button(self.f_left, text="Left", command=lambda: 
			self.move('L', trajectory_res,
				minimum_distance, ideal_color, step_color))
		self.b_forward = tk.Button(self.f_forwardbackward, text="Forward", 
			command=lambda: self.move('F', 
			trajectory_res, minimum_distance, ideal_color, step_color))
		self.b_backward = tk.Button(self.f_forwardbackward, text="Backward", 
			command=lambda: self.move('B', 
			trajectory_res, minimum_distance, ideal_color, step_color))
		self.b_up.pack() 
		self.b_down.pack() 
		self.b_right.pack() 
		self.b_left.pack() 
		self.b_forward.grid(row=0,column=0) 
		self.b_backward.grid(row=0,column=1)

		# Directional Movement Distance Specification Box
		self.distance_frame = tk.Frame(self.frame, height=100, width=100)
		self.distance_frame.grid(row=4,column=1)
		self.distance_labelText = StringVar()
		self.distance_labelText.set("Directional Distance to Move (mm)")
		self.distance_labelDir = Label(self.distance_frame, 
			textvariable=self.distance_labelText, height=1)
		self.distance_labelDir.grid(row=0,column=0)
		self.distance_directory = StringVar(None)
		self.distance_dirname = Entry(self.distance_frame, 
			textvariable=self.distance_directory, width=20)
		self.distance_dirname.grid(row=1,column=0)
		self.distance_dirname.insert(0,"10")

		# Slider for step input
		self.speed_frame = tk.Frame(self.frame, height=100, width=100)
		self.speed_frame.grid(row=0,column=1)
		self.speed_labelText = StringVar()
		self.speed_labelText.set("Speed (mm/sec)")
		self.speed_labelDir = Label(self.speed_frame, 
			textvariable=self.speed_labelText, height=1)
		self.speed_labelDir.grid(row=0,column=1)
		self.speed_directory = StringVar(None)
		self.step_slider = tk.Scale(self.speed_frame, from_=1, to=10, length=600, 
			tickinterval=1 ,orient=HORIZONTAL)
		self.step_slider.grid(row=1, columnspan=3)

		# Buttons for the selection of Polar travel vs. Cartesian travel
		self.travel_frame = tk.Frame(self.frame, height=100, width=100)
		self.travel_frame.grid(row=5,column=1)
		self.radio_xyz = tk.Radiobutton(self.travel_frame, text="XYZ",
			variable=self.travel_mode, value=0,command=lambda:
			self.change_travel_mode(0))
		self.radio_polar = tk.Radiobutton(self.travel_frame, text="Polar",
			variable=self.travel_mode, value=1,command=lambda:
			self.change_travel_mode(1))
		self.radio_xyz.grid(row=0, column=0)
		self.radio_polar.grid(row=0, column=1)

		# Frame for X, Y, Z target coordinates
		self.XYZ_frame = tk.Frame(self.frame, height=100, width=100)
		self.XYZ_frame.grid(row=6, columnspan=3)
		
		# Button for targeted movement execution
		self.go_button = tk.Button(self.XYZ_frame, text="Go", command=lambda: 
			self.move('G', trajectory_res,
				minimum_distance, ideal_color, step_color))
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

		# Position map (drawn points on canvas with polar grid background)
		self.grid_PIL = Image.open(self.grid_path)
		self.grid_image = ImageTk.PhotoImage(self.grid_PIL)
		self.canvas = tk.Canvas(self.frame, width=self.grid_PIL.width, 
			height=self.grid_PIL.height)
		self.canvas.grid(row=7, columnspan=3)
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
		
		# Point and click target setting

	def move(self, direction, resolution, d_threshold,
		ideal_color, step_color):
		# Disable all other buttons while camera travels to destination
		self.disable_screen()

		# Initializations
		remainder_R = 0; remainder_Theta = 0; remainder_Z = 0
		clear_canvas = True
		travel_mode = self.travel_mode
		ds_array = []
		res = resolution
		vel = self.step_slider.get()

		# Acquire current x, y, z position from encoded string
		[R, Theta, Z] = cf.get_current_position(self.position)
		[x, y, z] = cf.cyl_to_xyz(R, Theta, Z)
		ideal_x = self.ideal_position[0]
		ideal_y = self.ideal_position[1]
		ideal_z = self.ideal_position[2]

		if(direction=='G'):
			target = [np.array(self.acquire_target_pos())]
		else:
			# Acquire distance to move from slider
			try: 
				distance = float(self.distance_dirname.get())
			except:
				if self.distance_dirname.get() == "":
					distance = 0
				else:
					self.distance_dirname.delete(0,END)
					raise ValueError('Directional Distance to Move must ' + \
						'be a real number.')
			
			if(distance!=0):
				# Determine the target location
				if direction == 'U':
					target = [[ideal_x, ideal_y, ideal_z + distance]]
				elif direction == 'D':
					target = [[ideal_x, ideal_y, ideal_z - distance]]
				elif direction == 'R':
					target = [[ideal_x + distance, ideal_y, ideal_z]]
				elif direction == 'L':
					target = [[ideal_x - distance, ideal_y, ideal_z]]
				elif direction == 'F':
					target = [[ideal_x, ideal_y + distance, ideal_z]]
				elif direction == 'B':
					target = [[ideal_x, ideal_y - distance, ideal_z]]
			# if scale was zero, no steps will be computed
			else: 
				print("No movement was requested.\n")
				return

		print("Target = (" + str(target[0][0]) + ", " + str(target[0][1]) + 
			", " + str(target[0][2]) + ")")

		dR_polar = cf.xyz_to_cyl(target[0][0],target[0][1],target[0][2])[0] - R
		dTheta_polar = cf.xyz_to_cyl(target[0][0],target[0][1],
			target[0][2])[1] - Theta

		if((not travel_mode) | (direction!='G')):
			avoid_positive_x_axis = cf.check_path_for_x_wall(ideal_x, ideal_y,
				target[0][0], target[0][1])
			if(avoid_positive_x_axis):
				if(direction=='G'):
					# 2 legs to/from origin: to origin -> to target
					target = np.array([[0,0,z+(target[0][2]-z)/2],
						[target[0][0],target[0][1],target[0][2]]])
				else:
					# 3 legs at 90 angle: to y axis, to (0,y_target), to target
					target = np.array([[0,y,z+(target[0][2]-z)/3],
						[0,target[0][1],z+2*(target[0][2]-z)/3],[target[0][0],
						target[0][1],target[0][2]]])

		for i in range(np.shape(target)[0]):
			target_reached = False; skip = False
			res = resolution

			x_target = target[i][0]
			y_target = target[i][1]
			z_target = target[i][2]

			# Continue to compute dR, dTheta, dZ until target is "reached"
			while not (target_reached | skip):

				print("This is iteration " + str(resolution - res))
				print("x = " + str(x) + " y = " + str(y) + " z = " + str(z))
				
				if res == 0:
					dx = (x_target - x)
					dy = (y_target - y)
					dz = (z_target - z)
				else:
					# Calculate next leg of trajectory in cartesian space
					dx = (x_target - x)/res
					dy = (y_target - y)/res
					dz = (z_target - z)/res

				# Check if target has been reached
				d = math.sqrt((dx*res)**2 + (dy*res)**2 + (dz*res)**2)
				if (d < d_threshold):
					target_reached = True
					# Notify user when target has been reached
					print("\nTarget reached. Input another command, or " + \
						"press 'Exit' to close the window.\n")
					skip = True
					continue

				if res == 0:
					target_reached = True
					print("\nA terminal point was reached based on the " + \
						"specified resolution")
					skip = True
					continue

				# Calculate dR, dTheta without using a rotation matrix
				[dR, dTheta] = cf.nonMatrix_calc_dR_dTheta(x, y, z, dx, dy, dz, res)
				if((travel_mode) & (direction == 'G')):
					dR = dR_polar/resolution
					dTheta = dTheta_polar/resolution

				# Calculate the ideal number of steps suggested by coord transform
				R_steps = float(dR)/self.MM_PER_STEP_DR
				Theta_steps = float(dTheta)/self.RADS_PER_STEP_DT
				Z_steps = float(dz)/self.MM_PER_STEP_DZ
				[R_steps, Theta_steps, Z_steps, remainder_R, remainder_Theta,
				remainder_Z] = cf.calculate_steps(R_steps, Theta_steps, Z_steps,
				remainder_R, remainder_Theta, remainder_Z)

				# Finalize wait_time calculation
				total_steps = R_steps + Theta_steps + Z_steps
				wait_dR = abs(R_steps*self.MM_PER_STEP_DR)
				wait_dTheta = abs(Theta_steps*self.RADS_PER_STEP_DT)
				wait_dZ = abs(Z_steps*self.MM_PER_STEP_DZ)
				ds = math.sqrt(wait_dR**2+(R*wait_dTheta)**2+wait_dZ**2)
				wait = 1E3*(ds/vel - self.execution_time_per_step*total_steps)
				print("Wait time is: " + str(wait))
				if(wait<0):
					wait = 0

				# Compute ideal movement to check accuracy 
				# of trajectory following algorithm
				ideal_dx = (x_target - ideal_x) / (res)
				ideal_dy = (y_target - ideal_y) / (res)
				ideal_dz = (z_target - ideal_z) / (res)
				ideal_x = round(ideal_x + ideal_dx,3)
				ideal_y = round(ideal_y + ideal_dy,3)
				ideal_z = round(ideal_z + ideal_dz,3)

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
				self.position = [R_check, Theta_check, Z_check]
				self.ideal_position = [ideal_x, ideal_y, ideal_z]

				# Acquire x, y, z position from encoded string for next loop
				[R, Theta, Z] = cf.get_current_position(self.position)
				[x, y, z] = cf.cyl_to_xyz(R, Theta, Z)

				# Comment this out for infinte horizon gradient descent method
				res -= 1

		# Re-enable buttons when destination is reached
		self.enable_screen()


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
		self.canvas.create_oval(self.canvas.winfo_width()/2-2+new_x,
			self.canvas.winfo_height()/2-2-new_y,self.canvas.winfo_width()/2
			+2+new_x,self.canvas.winfo_height()/2+2-new_y,outline=ideal_color,
			fill=ideal_color)

		# Create new point by moving according to step_x, step_y, step_z
		new_step_x = scale_factor*step_pos[0]
		new_step_y = scale_factor*step_pos[1]
		new_step_z = scale_factor*step_pos[2]
		self.canvas.create_oval(self.canvas.winfo_width()/2-2+new_step_x,
			self.canvas.winfo_height()/2-2-new_step_y,self.canvas.winfo_width()
			/2+2+new_step_x,self.canvas.winfo_height()/2+2-new_step_y,
			outline=step_color,fill=step_color)
		
		# Refresh canvas
		self.canvas.update()

		return False


	def acquire_target_pos(self):
		# Handle the empty entry case (empty = 0)
		try: 
			x = float(self.X_dirname.get())
		except:
			if self.X_dirname.get() == "":
				x = 0
			else:
				self.X_dirname.delete(0,END)
				self.enable_screen()
				raise ValueError('Target X Coordinate must be a real number.')
		try: 
			y = float(self.Y_dirname.get())
		except:
			if self.Y_dirname.get() == "":
				y = 0
			else:
				self.Y_dirname.delete(0,END)
				self.enable_screen()
				raise ValueError('Target Y Coordinate must be a real number.')
		try: 
			z = float(self.Z_dirname.get())
		except:
			if self.Z_dirname.get() == "":
				z = 0
			else:
				self.Z_dirname.delete(0,END)
				self.enable_screen()
				raise ValueError('Target Z Coordinate must be a real number.')

		# Print selected target in mm to the terminal for user validation
		print("Target has the following coordinates (X, Y, Z): (" +
			str(x) + ", " + str(y) + ", " + str(z) + ")")

		return [x, y, z]


	def exit_program(self):
		self.destroy()


	def change_travel_mode(self,mode):
		self.travel_mode = mode


	def enable_screen(self):
		self.b_up.config(state=NORMAL)
		self.b_down.config(state=NORMAL)
		self.b_right.config(state=NORMAL)
		self.b_left.config(state=NORMAL)
		self.b_forward.config(state=NORMAL)
		self.b_backward.config(state=NORMAL)
		self.go_button.config(state=NORMAL)
		self.step_slider.config(state=NORMAL)
		self.radio_xyz.config(state=NORMAL)
		self.radio_polar.config(state=NORMAL)

	def disable_screen(self):
		self.b_up.config(state=DISABLED)
		self.b_down.config(state=DISABLED)
		self.b_right.config(state=DISABLED)
		self.b_left.config(state=DISABLED)
		self.b_forward.config(state=DISABLED)
		self.b_backward.config(state=DISABLED)
		self.go_button.config(state=DISABLED)
		self.step_slider.config(state=DISABLED)
		self.radio_xyz.config(state=DISABLED)
		self.radio_polar.config(state=DISABLED)




