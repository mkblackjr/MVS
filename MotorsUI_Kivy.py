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
import datetime
import video_stream
from camera import Camera
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

LabelBase.register(name="Meteora",fn_regular="./Fonts/Meteora.ttf")
LabelBase.register(name="Gtek",fn_regular="./Fonts/Gtek.ttf")
LabelBase.register(name="Zian",fn_regular="./Fonts/Zian.ttf")
LabelBase.register(name="Carrinady",fn_regular="./Fonts/Carrinady.ttf")
LabelBase.register(name="Nebulous",fn_regular="./Fonts/NebulousContent.otf")
LabelBase.register(name="Subzero",fn_regular="./Fonts/SubZER0.ttf")
LabelBase.register(name="EarthOrbiter",fn_regular="./Fonts/earthorbiterbold.ttf")
LabelBase.register(name="ElectromagneticLungs",fn_regular="./Fonts/ElectromagneticLungs.otf")
LabelBase.register(name="Xolonium",fn_regular="./Fonts/Xolonium-Bold.otf")

Config.set('kivy', 'exit_on_escape', '0')
Config.set('graphics', 'width', '900')
Config.set('graphics', 'height', '600')
Config.set('graphics','resizable',0)
Config.set('graphics','position','custom')
Config.set('graphics','left',250)
Config.set('graphics','top',10)

lock = threading.Lock()

###############################################################################
################################# MVS Manager ################################
###############################################################################

''' Functionality to Debug:

- Cannot change target textbox values
- XYZ Travel mode non-functional
- Popup when target reached/minimum distance met

'''

class MVS():
    # Class Variables
    _r             = 0
    _theta         = 0
    _z             = 0
    _pos           = [_r,_theta,_z]
    _idealPos      = _pos
    _vel           = 1
    _currentTarget = [0,0,0]
    _travelMode    = None
   
    # Class Objects
    _polarMap  = ObjectProperty(None)
    _videoFeed = ObjectProperty(None)

    def __init__(self,port=None,lock=None,**kwargs):
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
        while self._guiOpen:
            with lock:
                if not targetQueue.empty():
                    target = targetQueue.get()
                    go = True
                else: go = False
            if go:
                for i in range(len(target)):
                    try:
                        self._currentTarget[i] = float(target[i])
                    except ValueError:
                        self._currentTarget[i] = self._idealPos[i]
                        # Create popup screen to inform user of error
                        # OR when targetinput is highlighted disable all keys except numbers
                        pass
                self.update_data()
                self.move(self._currentTarget)
        try:
            self._serialPort.write(b'QUIT')
        except AttributeError: 
            pass
        except Exception as e:
            print(e)
        
        self.close()

    def update_data(self):
        id_list = self._app.root.ids.main_screen.ids
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H:%M:%S")

        # Target Tracking
        id_list.x_target.text = str(self._currentTarget[0])
        # id_list.x_target.padding_x = id_list.x_target.width / 2
        id_list.y_target.text = str(self._currentTarget[1])
        # id_list.y_target.padding_x = id_list.y_target.width / 2
        id_list.z_target.text = str(self._currentTarget[2])
        # id_list.z_target.padding_x = id_list.z_target.width / 2

        id_list.trajectory_log.text += (str(self._currentTarget[0:3]).replace(" ","") + \
            " " + timestamp + "\n")
        
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
            im = Image(source="./Images/polarGraph.png",size=(petri_id.height*5/6,
                petri_id.height*5/6),center=(petri_id.x + petri_id.width/2,
                petri_id.y + petri_id.height/2))
            petri_id.add_widget(im)
            
    def move(self,target):

        # Initializations
        remainderR = 0; remainderTheta = 0; remainderZ = 0
        clearCanvas = True
        travelMode = self._travelMode
        res = resolution
        vel = self._vel

        distance = 10 # Need to implement a way of specifying on UI

        # Acquire current x, y, z position
        [R, Theta, Z] = cf.get_current_position(self._pos)
        [x, y, z] = cf.cyl_to_xyz(R, Theta, Z)
        x_actual = x; y_actual = y; z_actual = z
        R_actual = R; Theta_actual = Theta;
        x_ideal = self._idealPos[0]
        y_ideal = self._idealPos[1]
        z_ideal = self._idealPos[2]

        start_time = time.time()
        R1 = R; Theta1 = Theta
        x1 = x; y1 = y; z1 = z

        # Determine the final target
        target = [np.array(target)]
        if len(target) == 4:
            directionalFlag = 1
            travelMode = 0
        else: directionalFlag = 0

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
                    end_time = time.time()
                    if travelMode:
                        mr = (R_actual-R1)/(end_time-start_time)
                        mtheta = (Theta_actual-Theta1)/(end_time-start_time)
                        mz = (Z_actual-z1)/(end_time-start_time)
                        r_parameterized = "r={}*(t-(t_start))+{}".format(mr,R1)
                        theta_parameterized = "theta={}*(t-(t_start))+{}".format(mtheta,Theta1)
                        z_parameterized = "z={}*(t-(t_start))+{}".format(mz,z1)
                        self._app.parametric_funcs.append([r_parameterized,
                            theta_parameterized,z_parameterized])
                    else:
                        mx = (x_actual-x1)/(end_time-start_time)
                        my = (y_actual-y1)/(end_time-start_time)
                        mz = (z_actual-z1)/(end_time-start_time)
                        x_parameterized = "x={}*(t-(t_start))+{}".format(mx,x1)
                        y_parameterized = "y={}*(t-(t_start))+{}".format(my,y1)
                        z_parameterized = "z={}*(t-(t_start))+{}".format(mz,z1)
                        self._app.parametric_funcs.append([x_parameterized,
                            y_parameterized,z_parameterized])
                    targetReached = True
                    # Notify user when target has been reached
                    self._app._targetReached("reached")
                    skip = True
                    continue

                # In case that target is not reached in time (shouldn't happen
                # numerically, maybe if trouble handling approaching zero) 
                if res == 0:
                    end_time = time.time()
                    x2 = x_actual; y2 = y_actual; z2 = z_actual
                    targetReached = True
                    self._app._targetReached("res")
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
                if(wait<0):
                    wait = 0

                if self._serialPort != None:
                    # Wait for the go ahead signal from arduino
                    sf.wait_for_arduino(self._serialPort)

                    # # Write computed integer steps to serial port in encoded form
                    # # 'R___T___Z___'
                    toWrite = cf.encode_step_command(R_steps, Theta_steps,
                        Z_steps, int(wait))
                    self._serialPort.write(toWrite)

                # Compute ideal movement to check accuracy 
                # of trajectory following algorithm
                ideal_dx = (x_target - x_ideal) / (res)
                ideal_dy = (y_target - y_ideal) / (res)
                ideal_dz = (z_target - z_ideal) / (res)
                x_ideal = round(x_ideal + ideal_dx,3)
                y_ideal = round(y_ideal + ideal_dy,3)
                z_ideal = round(z_ideal + ideal_dz,3)

                # Compute actual movement based on steps taken
                R_actual = R + R_steps*MM_PER_STEP_DR
                Theta_actual = Theta + Theta_steps*RADS_PER_STEP_DT
                Z_actual = Z + Z_steps*MM_PER_STEP_DZ
                [x_actual, y_actual, z_actual] = cf.cyl_to_xyz(R_actual,
                    Theta_actual, Z_actual)

                # Draw the points on the canvas
                origin = [self._app.root.ids.main_screen.ids.petri_map.center_x,
                          self._app.root.ids.main_screen.ids.petri_map.center_y]
                actual_point = [x+y for x,y in zip(origin,[10*x_actual,10*y_actual])]
                self.update_petri_canvas(actual_point,clearCanvas)
                clearCanvas = False

                # Update position to reflect steps taken and encode it properly
                self._pos = [R_actual, Theta_actual, Z_actual]
                self._idealPos = [x_ideal, y_ideal, z_ideal]

                # Update parameterized functions


                # Acquire x, y, z position from encoded string for next loop
                [R, Theta, Z] = [R_actual, Theta_actual, Z_actual]
                [x, y, z] = [x_actual, y_actual, z_actual]

                # Comment this out for infinte horizon gradient descent method
                res -= 1       

    def close(self):
        try:
            self._serialPort.close()
        except AttributeError: 
            pass
        except Exception as e:
            print(e)
        
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
from kivy.uix.label import Label
from kivy.clock import Clock
from kivy.uix.popup import Popup

############################################################################################################ Application Objects #############################
###############################################################################

class VideoFeed(Widget):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)    

class PetriMap(Widget):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)


class DirectionalArrowButtons(AnchorLayout):
    def __init__(self,lock=None,**kwargs):
        super().__init__(**kwargs)
        self.app = App.get_running_app()
        self._length = 10

    def move_directionally(self,x=0,y=0,z=0):
        newTarget = [x+y for x,y in zip(self.app._mvs._currentTarget,[x,y,z])]
        newTarget.append(True)
        with lock:
            self.app.commandQueue.put(newTarget)

    @property
    def length(self):
        return self._length


class MainScreen(Screen):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.app = App.get_running_app()

    def queueTarget(self):
        box = self.app.root.ids.main_screen.ids.movement_pane.ids
        target = [box.x_target.text,box.y_target.text,box.z_target.text]
        for i in range(len(target)):
            if target[i] == "":
                target[i] = 0.0
        with lock:
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
    parametric_funcs = [["f(x(t))","f(y(t))","f(z(t))"]]

    def __init__(self,port=None,**kwargs):
        # super(testApp,self).__init__(**kwargs)
        super().__init__(**kwargs)
        self.serial_port = port
    
    def build(self):
        self._gui = GUI()
        self._mvs = MVS(self.serial_port)

        return self._gui

    def on_start(self):
        if self._mvs.start():
            comThread = threading.Thread(name="ComThread",target=self._mvs.run,
                args=(self.commandQueue,True,))
            comThread.start()
            print("COM Thread Started")
            
            # microscope_vid_thread = threading.Thread(name="microscope_vid_thread",
            #     target=video_stream.serve,args=(0,3142,))
            # microscope_vid_thread.daemon = True
            # microscope_vid_thread.start()

            # petridish_vid_thread = threading.Thread(name="petridish_vid_thread",
            #     target=video_stream.serve,args=(1,3141,))
            # petridish_vid_thread.daemon = True
            # petridish_vid_thread.start()

        # Arduino code ensures motors are homed at startup
        else: print("Something went wrong during \"on_start\"")
    
    def on_stop(self):
        # Arduino code homes motors at shutdown
        self._mvs._guiOpen = False

    def _targetReached(self,mode):
        if mode == "reached":
            popup = Popup(title="Target Reached",
                content=Label(text="Target Reached!"),size_hint=(1/4,1/3))
            popup.open()
            time.sleep(1)
            popup.dismiss()
        elif mode == "res":
            popup = Popup(title="Target Not Reached",content=Label(text=
                "Target could not be reached within allotted discretization."),
                size_hint=(1/4,1/3))
            popup.open()
            time.sleep(1)
            popup.dismiss()

    def _view_microscope(self):
        layout = BoxLayout(orientation='vertical')
        lbl = Label(size_hint=(1,0.2),text="Microscope View")
        vid = AsyncImage(source='http://localhost:3141/current_image')
        layout.add_widget(lbl)
        layout.add_widget(vid)
        popup = Popup(title="Microscope View",content=layout,size_hint=(2/3,2/3))
        btn = Button(size_hint=(0.5,0.2),text="Exit",on_release=popup.dismiss())
        layout.add_widget(btn)
        
        popup.open()

    def _save(self,filename):
        filename += "/" + datetime.datetime.now().strftime("%Y%m%d_%H%M") + ".txt"
        log = self._gui.ids.main_screen.ids.trajectory_log
        with open(filename,'w') as f:
            f.write("Target t_start {} {} {}\n".format(
                self.parametric_funcs[0][0],self.parametric_funcs[0][1],
                self.parametric_funcs[0][2]))
            for line,func in zip(log.text.splitlines(),self.parametric_funcs[1:]):
                l = line + "  " +' '.join(func) + "\n"
                f.write(l)

    def _import(self,file):
        with open(file,'r') as f:
            r = csv.reader(f)
            for row in r:
                with lock:
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
    MotorsUI_App(arduino).run()
    # testApp().run()

    print(end)


