import serial
import time

arduino = serial.Serial('/dev/cu.usbmodem1411', 9600)

def onOffFunction():
	command = input("Type something..: (on / off / bye)")
	if command == "on":
		print ("The LED is on...")
		time.sleep(1) 
		arduino.write(13,HIGH) 
		onOffFunction()
	elif command == "off":
		print ("The LED is off...")
		time.sleep(1) 
		arduino.write(b'13',b'LOW')
		onOffFunction()
	elif command == "bye":
		print ("See You!...")
		time.sleep(1) 
		arduino.close()
	else:
		print ("Sorry..type another thing..!")
		onOffFunction()


time.sleep(2)
onOffFunction()