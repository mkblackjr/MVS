# controlMotors.py

""" 
Motor Control Program

This program opens a user interface for motor control and directs the flow of
communication to the Arduino for command processing and eventual motor movement.

"""

from motorsUI import motorsUI as ui
import time
import string
import serial

###############################################################################
################################## Settings ###################################
###############################################################################
scale_factor = 10
step_increment = 10
resolution = 20
exact_color = "#ff0000" # Red
step_color = "#0000cd" # Medium Blue
minimum_distance = 0.5

# Establish serial connection with the Arduino/Genuino Uno
connected = False
# arduino = "no" 
arduino = serial.Serial("/dev/cu.usbmodem1411", 9600)
time.sleep(3) # Allow connection to be established

# Verify connection with the Arduino via messages sent and received
while not connected:
    arduino.write(b'We are live.\n')
    time.sleep(1)
    feedback = arduino.readline().decode("utf-8")
    print(feedback)
    connected = True

command_window = ui(arduino, scale_factor, step_increment, resolution,
	minimum_distance, exact_color, step_color)
command_window.mainloop()

# arduino.close()

###############################################################################
