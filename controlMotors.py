# controlMotors.py

""" 
Motor Control Program

This program opens a user interface for control of the motors of the MVSS and 
directs the flow of communication to the Arduino via serial communication for 
command processing and motor movement.

"""

# from NoSerialMotorUI import motorsUI as ui
from motorsUI import motorsUI as ui
import SerialFunctions as sf
import time
import string
import serial

###############################################################################
################################## Settings ###################################
###############################################################################

scale_factor = 2
step_increment = 10
resolution = 100
exact_color = "#ff0000" # Red
step_color = "#0000cd" # Medium Blue
minimum_distance = 0.05

###############################################################################
############################### Initialization ################################
###############################################################################

# Establish serial connection with the Arduino/Genuino Uno
connected = False
arduino = "no"
arduino = serial.Serial("/dev/cu.usbmodem1411", 9600)
time.sleep(2) # Allow connection to be established

###############################################################################
##################################### Run #####################################
###############################################################################

# Verify connection with the Arduino via messages sent and received
while not connected:
    arduino.write(b'We are live.\n')
    time.sleep(1)
    feedback = arduino.readline().decode("utf-8")
    print(feedback)
    connected = True
    sf.wait_for_arduino(arduino)
    print(arduino.readline().decode("utf-8"))

command_window = ui(arduino, scale_factor, step_increment, resolution,
	minimum_distance, exact_color, step_color)
command_window.mainloop()

arduino.close()

###############################################################################
###############################################################################
###############################################################################
