# velocityTest.py

from motorsUI import motorsUI as ui
import SerialFunctions as sf
import time
import string
import serial
import csv

###############################################################################
################################## Settings ###################################
###############################################################################

# Establish serial connection with the Arduino/Genuino Uno
connected = False
arduino = serial.Serial("/dev/cu.usbmodem1411", 9600)
time.sleep(2) # Allow connection to be established
arduino_time = []

# Verify connection with the Arduino via messages sent and received
while not connected:
    arduino.write(b'We are live.\n')
    time.sleep(1)
    feedback = arduino.readline().decode("utf-8")
    print(feedback)
    connected = True
    sf.wait_for_arduino(arduino)
    print(arduino.readline().decode("utf-8"))

time.sleep(2)

arduino.write(b'R500T0Z0W0')
start_time = time.time()
sf.wait_for_arduino(arduino)
end_time = time.time() - start_time
arduino_time.append(end_time)

arduino.write(b'R0T500Z0W0')
start_time = time.time()
sf.wait_for_arduino(arduino)
end_time = time.time() - start_time
arduino_time.append(end_time)

arduino.write(b'R0T0Z500W0')
start_time = time.time()
sf.wait_for_arduino(arduino)
end_time = time.time() - start_time
arduino_time.append(end_time)

arduino.write(b'R-500T0Z0W0')
start_time = time.time()
sf.wait_for_arduino(arduino)
end_time = time.time() - start_time
arduino_time.append(end_time)

arduino.write(b'R0T-500Z0W0')
start_time = time.time()
sf.wait_for_arduino(arduino)
end_time = time.time() - start_time
arduino_time.append(end_time)

arduino.write(b'R0T0Z-500W0')
start_time = time.time()
sf.wait_for_arduino(arduino)
end_time = time.time() - start_time
arduino_time.append(end_time)

# Speed test code
arduino_message = ["\nThe following data represents the loop time " + \
	" in seconds for the Arduino pins to activate and move."]
csvfile = "/Users/mac/Documents/MVSSgit/Velocity_test.txt"
with open(csvfile, "a") as output:
	writer = csv.writer(output, lineterminator='\n')
	writer.writerow(arduino_message)
	for val in arduino_time:
		writer.writerow([val])

arduino.close()

###############################################################################
