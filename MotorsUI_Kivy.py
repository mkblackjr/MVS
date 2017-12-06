#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 27 11:45:14 2017

@author: mac
"""
###############################################################################
################################ Global Imports ###############################
###############################################################################

import CoordinateFunctions as cf
import SerialFunctions as sf
import MoveFunctions as move
import numpy as np
import serial
import queue
import threading
import time
import csv
from math import pi,sqrt
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.app import App
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty, StringProperty, BooleanProperty
from kivy.graphics import Color,Ellipse,Rectangle
from kivy.uix.image import Image

###############################################################################
############################### Program Settings ##############################
###############################################################################

# Physical parameters
in_to_mm = 25.4
steps_per_rev_motor = 4096
rads_per_step_motor = 2*pi/steps_per_rev_motor
gear_diameter = 20 # mm
planetary_gear_diameter = 5.25*in_to_mm # mm
gear_ratio = gear_diameter/planetary_gear_diameter
motor_step_correction_factor = 510/4096
# MM_PER_STEP_DZ = 0.0062 # Number of milimeters per step for DR
# RADS_PER_STEP_DT = 0.00147 # Number of rads per step for DTheta
# MM_PER_STEP_DR = 0.123 # Number of milimeters per step for DZ
MM_PER_STEP_DZ = 0.0062*motor_step_correction_factor
RADS_PER_STEP_DT = rads_per_step_motor*gear_ratio
MM_PER_STEP_DR = gear_diameter*pi/steps_per_rev_motor
execution_time_per_step = 0.0010664

# Path Planning Parameters
d_threshold = 0.05
resolution = 100
# scale_factor = 2
# step_increment = 10
# resolution = 100
# exact_color = "#ff0000" # Red
# step_color = "#0000cd" # Medium Blue
end = "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"

###############################################################################
################################ Window Settings ##############################
###############################################################################

from kivy.core.text import LabelBase
from kivy.config import Config

# Config.read('/Users/mac/.kivy/config.ini')
# Config.set('input','mouse','mouse,multitouch_on_demand')
# Config.write()

LabelBase.register(name="Meteora",fn_regular="Meteora.ttf")
LabelBase.register(name="Gtek",fn_regular="Gtek.ttf")
LabelBase.register(name="Zian",fn_regular="Zian.ttf")

Config.set('graphics', 'width', '900')
Config.set('graphics', 'height', '600')
# Config.set('graphics','borderless',1)
Config.set('graphics','resizable',0)
Config.set('graphics','position','custom')
Config.set('graphics','left',250)
Config.set('graphics','top',10)

###############################################################################
################################# MVSS Manager ################################
###############################################################################

class MVSS():
    # Class Variables
    _x = 0
    _y = 0
    _z = 0
    _pos = [_x,_y,_z]
    _idealPos = _pos
    _vel = 1
    _currentTarget = [0,0,0]
    _travelMode = None
   
    # Class Objects
    _polarMap = ObjectProperty(None)
    _videoFeed = ObjectProperty(None)

    def __init__(self,port=None,**kwargs):
        super().__init__(**kwargs)
        self._serialPort = port
        self._app = None
        self._connected = False
        self._guiOpen = False      

    def start(self):
        self._app = App.get_running_app()
        self._main_ids = self._app.root.ids.main_screen.ids
        self._save_ids = self._app.root.ids.save_screen.ids
        self._import_ids = self._app.root.ids.import_screen.ids
        self._quit_ids = self._app.root.ids.quit_screen.ids

        return self.open_serial_port()

    def open_serial_port(self):

        if self._serialPort != None:
            # Establish Connection
            while not self._connected:
                self._serialPort.write(b'We are live.\n')
                time.sleep(1)
                feedback = arduino.readline().decode("utf-8")
                print(feedback)
                # if (feedback == "Go\n"):
                if True:
                    self._connected = True

        return True
        return self._connected

    def run(self,targetQueue,status):
        self._guiOpen = status
        # While GUI is open
        print(self._guiOpen)
        while self._guiOpen:
            if not targetQueue.empty():
                target = targetQueue.get()
                for i in range(len(target)):
                    self._currentTarget[i] = float(target[i])
                self.update_data()
                self.move(self._currentTarget)
        # self._serialPort.write(b'QUIT')
        self.close()

    def update_data(self):
        id_list = self._app.root.ids.main_screen.ids

        # Target Tracking
        id_list.trajectory_log.text += (str(self._currentTarget[0:3]) + "\n")
        id_list.x_target.text = str(self._currentTarget[0])
        # id_list.x_target.padding_x = id_list.x_target.width / 2
        id_list.y_target.text = str(self._currentTarget[1])
        # id_list.y_target.padding_x = id_list.y_target.width / 2
        id_list.z_target.text = str(self._currentTarget[2])
        # id_list.z_target.padding_x = id_list.z_target.width / 2

    def update_petri_canvas(self,position,clear):
        petri_id = self._app.root.ids.main_screen.ids.petri_map
        if not clear:
            with petri_id.canvas:
                    Color(0.4,0.65,1)
                    Ellipse(pos = position,size=(2,2))
        else:
            with petri_id.canvas:
                Color(1,1,1)
                Ellipse(pos=(petri_id.x + petri_id.width/2 - 
                    5*petri_id.height/12,petri_id.y + petri_id.height/12),
                    size=(petri_id.height*5/6,petri_id.height*5/6))
            im = Image(source="polarGraph.png",size=(petri_id.height*5/6,
                petri_id.height*5/6),center=(petri_id.x + petri_id.width/2,
                petri_id.y + petri_id.height/2))
            petri_id.add_widget(im)
            
            # with petri_id:

    def move(self,target):

        # Initializations
        remainderR = 0; remainderTheta = 0; remainderZ = 0
        clearCanvas = True
        travelMode = (self._travelMode != "XYZ")
        res = resolution
        vel = self._vel

        distance = 10 # Need to implement a way of specifying on UI

        # Acquire current x, y, z position
        [R, Theta, Z] = cf.get_current_position(self.position)
        [x, y, z] = cf.cyl_to_xyz(R, Theta, Z)
        x_ideal = self.ideal_position[0]
        y_ideal = self.ideal_position[1]
        z_ideal = self.ideal_position[2]

        # Determine the final target
        target = [np.array(target)]
        if len(target) == 4:
            directionalFlag = 1
            travelMode = 0
        else: directionalFlag = 0

        print("Target = (" + str(target[0][0]) + ", " + str(target[0][1]) + 
            ", " + str(target[0][2]) + ")")

        # Compute dR, dTheta for case of polar travel mode
        (dR_polar,dTheta_polar,_) = tuple(np.subtract(cf.xyz_to_cyl(target[0][0],
            target[0][1],target[0][2]),[R,Theta,0]))

        # Update target to incorporate wall at Theta = 0/2pi
        target = cf.update_target(target, y, z, x_ideal, y_ideal,
            travelMode, directionalFlag)

        # Iterate over the target array to hit waypoints
        for i in range(np.shape(target)[0]):
            targetReached = False; skip = False
            res = resolution

            x_target = target[i][0]
            y_target = target[i][1]
            z_target = target[i][2]

            # Continue to compute dR, dTheta, dZ until target is "reached"
            while not (targetReached | skip):

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
                d = sqrt((dx*res)**2 + (dy*res)**2 + (dz*res)**2)
                if (d < d_threshold):
                    targetReached = True
                    # Notify user when target has been reached
                    print("\nTarget reached.\n")
                    skip = True
                    continue

                # In case that target is not reached in time (shouldn't happen
                # numerically, maybe if trouble handling approaching zero) 
                if res == 0:
                    targetReached = True
                    print("\nA terminal point was reached based on the " + \
                        "specified resolution")
                    skip = True
                    continue

                # Calculate dR, dTheta without using a rotation matrix
                [dR, dTheta] = cf.nonMatrix_calc_dR_dTheta(x, y, z, dx, dy, dz, res)
                if travelMode:
                    dR = dR_polar/resolution
                    dTheta = dTheta_polar/resolution

                # Compute the ideal number of steps suggested by coord transform
                R_steps = float(dR)/MM_PER_STEP_DR
                Theta_steps = float(dTheta)/RADS_PER_STEP_DT
                Z_steps = float(dz)/MM_PER_STEP_DZ

                # Calculate steps to send to Arduino (with accumulated remainder)
                [R_steps, Theta_steps, Z_steps, remainderR, remainderTheta,
                remainderZ] = cf.calculate_steps(R_steps, Theta_steps, Z_steps,
                remainderR, remainderTheta, remainderZ)

                # Finalize wait_time calculation
                total_steps = R_steps + Theta_steps + Z_steps
                wait_dR = abs(R_steps*MM_PER_STEP_DR)
                wait_dTheta = abs(Theta_steps*RADS_PER_STEP_DT)
                wait_dZ = abs(Z_steps*MM_PER_STEP_DZ)
                ds = sqrt(wait_dR**2+(R*wait_dTheta)**2+wait_dZ**2)
                wait = 1E3*(ds/vel - execution_time_per_step*total_steps)
                print("Wait time is: " + str(wait))
                if(wait<0):
                    wait = 0

                # Wait for the go ahead signal from arduino
                # sf.wait_for_arduino(open_serial_port)

                # # Write computed integer steps to serial port in encoded form
                # # 'R___T___Z___'
                print(R_steps)
                print(Theta_steps)
                print(Z_steps)
                toWrite = cf.encode_step_command(R_steps, Theta_steps,
                    Z_steps, int(wait))
                print(toWrite)
                # open_serial_port.write(toWrite)

                # Compute ideal movement to check accuracy 
                # of trajectory following algorithm
                ideal_dx = (x_target - x_ideal) / (res)
                ideal_dy = (y_target - y_ideal) / (res)
                ideal_dz = (z_target - z_ideal) / (res)
                x_ideal = round(x_ideal + ideal_dx,3)
                y_ideal = round(y_ideal + ideal_dy,3)
                z_ideal = round(z_ideal + ideal_dz,3)

                print("The ideal trajectory is currently at (" + str(x_ideal) + \
                    ", " + str(y_ideal) + ", " + str(z_ideal) + ")\n")

                # Compute actual movement based on steps taken
                R_actual = R + R_steps*MM_PER_STEP_DR
                Theta_actual = Theta + Theta_steps*RADS_PER_STEP_DT
                Z_actual = Z + Z_steps*MM_PER_STEP_DZ
                [x_actual, y_actual, z_actual] = cf.cyl_to_xyz(R_actual,
                    Theta_actual, Z_actual)

                # Draw the points on the canvas
                origin = [self._app.root.ids.main_screen.ids.petri_map.center_x,
                          self._app.root.ids.main_screen.ids.petri_map.center_y]
                actual_point = [x+y for x,y in zip(origin,[x_actual,y_actual])]
                print(actual_point)
                self.update_petri_canvas(actual_point,clearCanvas)
                # self.update_canvas(scale_factor, [x_ideal, y_ideal, z_ideal], 
                #     [x_actual, y_actual, z_actual], ideal_color, step_color,
                #     clearCanvas)
                clearCanvas = False

                # Update position to reflect steps taken and encode it properly
                self.position = [R_actual, Theta_actual, Z_actual]
                self.ideal_position = [x_ideal, y_ideal, z_ideal]

                # Acquire x, y, z position from encoded string for next loop
                [R, Theta, Z] = [R_actual, Theta_actual, Z_actual]
                [x, y, z] = [x_actual, y_actual, z_actual]

                # Comment this out for infinte horizon gradient descent method
                res -= 1       

    def get_input(self):
        print("input")
        time.sleep(1)

    def close(self):
        # self._serialPort.close()
        pass
        
    @property
    def position(self):
        return self._pos
    @property
    def pos_str(self):
        return str(self._pos)
    @position.setter
    def position(self,pos):
        self._pos = pos

    @property
    def ideal_position(self):
        return self._idealPos
    @ideal_position.setter
    def ideal_position(self,iPos):
        self._idealPos = iPos

    @property
    def velocity(self):
        return self._vel
    @property
    def vel_str(self):
        return str(self._vel)
    @velocity.setter
    def velocity(self,vel):
        if vel < 1:
            vel = 1
        elif vel > 10:
            vel = 10
        self._vel = vel

###############################################################################
################################# Kivy Imports ################################
###############################################################################

from kivy.core.window import Window
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.widget import Widget
from kivy.clock import Clock

############################################################################################################ Application Objects #############################
###############################################################################

class VideoFeed(Widget):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
    

class PetriMap(Widget):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)


class DirectionalArrowButtons(AnchorLayout):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.app = App.get_running_app()
        self._length = 10

    def move_directionally(self,x=0,y=0,z=0):
        newTarget = [x+y for x,y in zip(self.app._mvs._currentTarget,[x,y,z])]
        newTarget.append(True)
        self.app.commandQueue.put(newTarget)

    @property
    def length(self):
        return self._length


class MainScreen(Screen):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.app = App.get_running_app()

    def queueTarget(self):
        box = self.app.root.ids.main_screen.ids
        target = [box.x_target.text,box.y_target.text,box.z_target.text]
        for i in range(len(target)):
            print(type(target[i]))
            if target[i] == "":
                target[i] = 0.0
        self.app.commandQueue.put(target)
        

class GUI(ScreenManager):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)

###############################################################################
############################### GUI Application ###############################
###############################################################################

# class testApp(App):
class MotorsUI_App(App):
    title = "MVS"
    commandQueue = queue.Queue()
    commandQueue.put([0,0,0])

    def __init__(self,port=None,**kwargs):
        # super(testApp,self).__init__(**kwargs)
        super().__init__(**kwargs)
        self.serial_port = port
    
    def build(self):
        self._gui = GUI()
        self._mvs = MVSS(self.serial_port)

        return self._gui

    def on_start(self):
        if self._mvs.start():
            self._comThread = threading.Thread(name="ComThread",target=self._mvs.run,args=(self.commandQueue,True,))
            self._comThread.start()
        # Arduino code ensures motors are homed at startup
    
    def on_stop(self):
        # Arduino code homes motors at shutdown
        self._mvs._guiOpen = False

    def _save(self,filename):
        mod = self._gui.ids.main_screen.ids.trajectory_log
        with open(filename,'wb') as f:
            w = csv.writer(f,delimiter=',')
            for line in mod.text:
                w.writerow(line)

    def _import(self,file):
        with open(file,'r') as f:
            r = csv.reader(f)
            for row in r:
                self.commandQueue.put(row)




###############################################################################
################################ Main Execution ###############################
###############################################################################

if __name__=="__main__":
    try:
        com_port = "/dev/cu.usbmodem1411"
        arduino = serial.Serial(com_port, 9600)
        time.sleep(3)
    except:
        com_port = None
        arduino = None
    print(arduino)
    MotorsUI_App(port=arduino).run()
    # testApp().run()

    print(end)




