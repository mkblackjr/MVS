# MotorCoordinateFunctions.py

""" 
Serial Port Related Functions

This program contains functions used in controlling the transmission and 
reception of data to/from the serial port connecting python and arduino in the
MVSS.

"""
import serial
import time
import CoordinateFunctions as cf

###############################################################################
############################ Function Definitions #############################
###############################################################################

def wait_for_arduino(serial_port):
	# Continue reading from serial port until arduino is ready
	ready = serial_port.readline().decode("utf-8")
	while(ready != "Go\n"):
		ready = serial_port.readline().decode("utf-8")
		print("waiting\n")