import math
import numpy as np

def move(serial_port,d_threshold):
    # Initializations
    remainder_R = 0; remainder_Theta = 0; remainder_Z = 0
    target_reached = False
    clear_canvas = True
    ideal_path_color = 1,0,0
    actual_path_color = 0,0,.8039

    # Acquire target position from entry boxes
    [x_target, y_target, z_target] = acquire_target_pos()

    # Acquire current x, y, z position from encoded string
    [R, Theta, Z] = cf.get_current_position(position)
    [x, y, z] = cf.cyl_to_xyz(R, Theta, Z)
    ideal_x = x; ideal_y = y; ideal_z = z

    iterations = 0

    # Continue to compute dR, dTheta, dZ until target is "reached"
    while not target_reached:
        
        wait_for_arduino(serial_port)

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
            serial_port.write(b"R0T0Z0\n")
            break

        if self.resolution == 0:
            target_reached = True
            print("\nA terminal point was reached based on the " + \
                "specified self.resolution")
            serial_port.write(b"R0T0Z0\n")
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
        serial_port.write(toWrite)

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
            check_y, check_z], ideal_path_color, actual_path_color, clear_canvas)
        clear_canvas = False

        # Update position to reflect steps taken and encode it properly
        self.position = "R" + str(R_check) + "T" + str(Theta_check) + \
        "Z" + str(Z_check) # line for debugging when not plugged into arduino

        # Acquire x, y, z position from encoded string for next loop
        [R, Theta, Z] = cf.get_current_position(self.position)
        [x, y, z] = cf.cyl_to_xyz(R, Theta, Z)

        # Comment this out for infinte horizon gradient descent method
        self.resolution -= 1


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