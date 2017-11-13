# from pyduino import *
from numpy import *
# import motor
import time
import string
import serial
# import threading
from math import cos, sin, atan2, sqrt, pi # For polar conversion.
#from polar import *
# from queue import Queue


# CURRENTLY ASSUMING:
#   Motor 1 is set up as R.
#   Motor 2 is set up as Theta.
#   Motor 3 is set up as Z.
#   Clockwise rotation always results in a move towards (0, 0, 0)

# Current set up of motor steps.

#     Pin 1   Pin 2   Pin 3   Pin 4
#  1    x
#  2    x       x
#  3            x
#  4            x       x
#  5                    x
#  6                    x      x
#  7                           x
#  8    x                      x



# Define the number of steps for the motor
# to complete a single full rotation.
FULL_CIRCLE = 510.0

# Number of millimeters per step for DR and DZ.
# Amount of radians per one step for DTHETA
STEPS_PER_MM_DZ = 0.0062
STEPS_PER_RAD_DT = 0.00147
STEPS_PER_MM_DR = 0.123

# Inverse relations.
DZ_TO_STEPS_PER_MM = 1 / STEPS_PER_MM_DZ
DT_TO_STEPS_PER_RAD = 1 / STEPS_PER_RAD_DT
DR_TO_STEPS_PER_MM = 1 / STEPS_PER_MM_DR

###############################################################################

# Establish serial connection with the Arduino/Genuino Uno
connected = False
arduino = serial.Serial("/dev/cu.usbmodem1411", 9600)
time.sleep(3) # Allow connection to be established

# Verify connection with the Arduino via messages sent and received
while not connected:
    arduino.write(b'We are live.\n')
    time.sleep(1)
    feedback = arduino.readline().decode("utf-8")
    print(feedback)
    connected = True

# Submit an inquiry
successful_inquiry = False
modified_inquiry = "\nI'm sorry, your selection was not among the given " + \
"options. Please try again."
exit_loop = False

while not exit_loop:

    inquiry = "\nWhat would you like to do?\n(UP / DOWN / RIGHT / LEFT / " + \
    "FORWARD / BACKWARD)\n "

    while not successful_inquiry:
        command = input(inquiry)
        successful_inquiry = True
        more_tasks = True

        if (command == "UP"):
            arduino.write(b'We are live.\n')
            move_up()
            time.sleep(2)
        elif (command == "DOWN"):
            move_down()
        elif (command == "RIGHT"):
            move_right()
        elif (command == "LEFT"):
            move_left()
        elif (command == "BACKWARD"):
            move_backward()
        elif (command == "FORWARD"):
            move_forward()
        elif modified_inquiry not in inquiry: 
            inquiry = modified_inquiry + inquiry
            successful_inquiry = False
        else: 
            successful_inquiry = False

    ask_again = "\nWould you like to make another movement?\n(YES / NO)\n"

    while more_tasks:
        more_tasks = False
        response = input(ask_again)
        if (response == "YES"):
            successful_inquiry = False
        elif (response == "NO"):
            exit_loop = True
        elif modified_inquiry not in ask_again:
            ask_again = modified_inquiry + ask_again
            more_tasks = True
        else: more_tasks = True





arduino.close()

