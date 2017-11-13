from pyduino import *
from numpy import *
import time
try:
    import RPi.GPIO as GPIO # Raspberry Pi I/O library
except:
    GPIO = False
import threading
from math import cos, sin, atan2, sqrt, pi # For polar conversion.
#from polar import *
from queue import Queue


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



# --------------------------- REFACTOR -----------------------------------------


class Motor(object):



    # ------------------- Set up ---------------------


    def __init__(self):
        # Set up for the Arduino
        
        # if your arduino was running on a serial port other than '/dev/ttyACM0/'
        # declare: a = Arduino(serial_port= [Specify true path here])
        self.arduino = Arduino()
    
        # sleep to ensure ample time for computer to make serial connection
        time.sleep(3)

        for i in range(1, 4):
            self.arduino.set_pin_mode(self.get_channels(i), 'O')

        # # Set up for the Raspberry Pi
        # if not GPIO: return
        # GPIO.setmode(GPIO.BOARD)
        # GPIO.setwarnings(False)

        # for i in range(1, 4):
        #     GPIO.setup(self.get_channels(i), GPIO.OUT)


    def move(self, trajectory):
        # Move the platform, given a list of steps for each motor.
        threads = []
        r_worker = threading.Thread(target=self.move_r, args=(trajectory[0]))
        theta_worker = threading.Thread(target=self.move_theta, \
                                                        args = (trajectory[1]))
        z_worker = threading.Thread(target=self.move_z, args = (trajectory[2]))

        threads.append(r_worker)
        threads.append(theta_worker)
        threads.append(z_worker)

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()

        # for steps in trajectory:
        #     self.move_r(steps[0])
        #     self.move_theta([steps[1]])
        #     self.move_z([steps[2]])


    def move_r(self, steps):
        # Move the R motor.
        channel = self.get_channels(1)
        if steps > 0:
            self._counter_turn(channel, abs(steps))
        else:
            self._clockwise_turn(channel, abs(steps))


    def move_theta(self, steps):
        # Move the theta motor.
        channel = self.get_channels(2)
        if steps < 0:
            self._counter_turn(channel, abs(steps))
        else:
            self._clockwise_turn(channel, abs(steps))


    def move_z(self, steps):
        # Move the Z motor.
        channel = self.get_channels(3)
        if steps > 0:
            z = self._counter_turn(channel, abs(steps))
        else:
            z = self._clockwise_turn(channel, abs(steps))


    def get_channels(self, motor):
            # Return the appropriate control channels
            # based on which motor is specified.
            if motor == 1:
                return [4, 5, 6, 7]
                # return [7, 11, 13, 15]
            elif motor == 2:
                return [12, 16, 18, 22]
            else: # Motor 3.
                return [29, 31, 33, 35]


    def run(self, num_steps, channels, direction):
        # Target for the worker threads.
        if direction == 'clock':
            self._clockwise_turn(channels, num_steps)
        else: # direction == 'counter'
            self._counter_turn(channels, num_steps)


    def _gpio_setup(self, channels, inputs):
            # Loop through the two lists simultaneously,
            # and apply the motor changes to each channel.
            for chan, step in zip(channels, inputs):
                # if step:
                #     self.arduino.bitSet(PORTB,chan)
                # else:
                #     self.arduino.bitClear(PORTB,chan)
                self.arduino.analog_write(chan, step*255)
                # GPIO.output(chan, step)
            # Allow motor to receive instructions
            print(inputs)
            time.sleep(0.001)


    def _clockwise_turn(self, channels, steps):
        # Turn the given motor right by a given number of degrees.
        self._gpio_setup(channels, [0, 0, 0, 0])
        self._gpio_setup(channels, [1, 0, 0, 0])
        current_step = [1, 0, 0, 0]

        while steps > 0.0:
            current_step = self._get_next_clockwise(current_step)
            self._gpio_setup(channels, current_step)
            steps -= 1

        self._gpio_setup(channels, [0, 0, 0, 0])



    def _counter_turn(self, channels, steps):
        # Turn the given motor left by a given number of degrees.
        self._gpio_setup(channels, [0, 0, 0, 0])
        self._gpio_setup(channels, [1, 0, 0, 1])
        current_step = [1, 0, 0, 1]

        while steps > 0.0:
            current_step = self._get_next_counter_clockwise(current_step)
            self._gpio_setup(channels, current_step)
            steps -= 1

        self._gpio_setup(channels, [0, 0, 0, 0])


    def _get_next_clockwise(self, current_step):
        # Big ugly function brute forcing the steps.  Find a better way!
        if current_step == [1, 0, 0, 0]:
            return [1, 1, 0, 0]
        elif current_step == [1, 1, 0, 0]:
            return [0, 1, 0, 0]
        elif current_step == [0, 1, 0, 0]:
            return [0, 1, 1, 0]
        elif current_step == [0, 1, 1, 0]:
            return [0, 0, 1, 0]
        elif current_step == [0, 0, 1, 0]:
            return [0, 0, 1, 1]
        elif current_step == [0, 0, 1, 1]:
            return [0, 0, 0, 1]
        elif current_step == [0, 0, 0, 1]:
            return [1, 0, 0, 1]
        else: # current_step == [1, 0, 0, 1]
            return [1, 0, 0, 0]


    def _get_next_counter_clockwise(self, current_step):
        # Big ugly function brute forcing the steps.  Find a better way!
        if current_step == [1, 0, 0, 1]:
            return [0, 0, 0, 1]
        elif current_step == [0, 0, 0, 1]:
            return [0, 0, 1, 1]
        elif current_step == [0, 0, 1, 1]:
            return [0, 0, 1, 0]
        elif current_step == [0, 0, 1, 0]:
            return [0, 1, 1, 0]
        elif current_step == [0, 1, 1, 0]:
            return [0, 1, 0, 0]
        elif current_step == [0, 1, 0, 0]:
            return [1, 1, 0, 0]
        elif current_step == [1, 1, 0, 0]:
            return [1, 0, 0, 0]
        else: # current_step == [1, 0, 0, 0]
            return [1, 0, 0, 1]




test = Motor()
test.move_r(5000)