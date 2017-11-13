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
import MoveFunctions as move
import serial
import queue
import threading
import time
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.app import App
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty, StringProperty, BooleanProperty

###############################################################################
############################### Program Settings ##############################
###############################################################################

com_port = "/dev/cu.usbmodem1411"
arduino = None
# arduino = serial.Serial(com_port, 9600)
# time.sleep(3)

###############################################################################
################################ Window Settings ##############################
###############################################################################

from kivy.config import Config
from kivy.core.text import LabelBase

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

class MVSS(ScreenManager):
    # Objects
    _pos = StringProperty(0)
    _vel = NumericProperty(0)
    _target = NumericProperty(0),NumericProperty(0),NumericProperty(0)
    _travelMode = BooleanProperty(None)
    _polarMap = ObjectProperty(None)
    _videoFeed = ObjectProperty(None)

    def __init__(self,port=None,**kwargs):
        super().__init__(**kwargs)
        self._serialPort = port
        self._app = None
        self._connected = False
        self._isInput = False        

    def start(self):
        self._app = App.get_running_app()
        self._main_ids = self._app.root.ids.main_screen.ids
        self._save_ids = self._app.root.ids.save_screen.ids
        self._import_ids = self._app.root.ids.import_screen.ids
        self._quit_ids = self._app.root.ids.quit_screen.ids

        return self.open_serial_port()

    def open_serial_port(self):
        # Establish Connection
        while not self._connected:
            # self._serialPort.write(b'We are live.\n')
            time.sleep(1)
            # feedback = arduino.readline().decode("utf-8")
            # print(feedback)
            self._connected = True

        return self._connected

    def run(self):
        while not self._isInput:
            self.get_input()

        self.move()

    def move(self):
        self.position = 1,2,3

    def get_input(self):
        print("input")
        time.sleep(1)

        
    @property
    def position(self):
        return self._pos
    @position.setter
    def position(self,pos):
        self._pos = pos

    @property
    def velocity(self):
        return self._vel
    @property
    def vel_str(self):
        return str(self._vel)
    @velocity.setter
    def velocity(self,vel):
        if vel < 0:
            vel = 0
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

    def right(self):
        self.app.root.move()
        print(self.app.root.position)
        
    def left(self):
        print("Move Left")
        
    def forward(self):
        print("Move Forward")
        
    def backward(self):
        print("Move Backward")
        
    def up(self):
        print("Move Up")
        
    def down(self):
        print("Move Down") 


# class MovementPane(BoxLayout):
#     def __init__(self,**kwargs):
#         super().__init__(**kwargs)


# class WelcomeScreen(Screen):


# class QuitScreen(Screen):
#     def __init__(self,**kwargs):
#         super().__init__(**kwargs)


# class SaveScreen(Screen):
#     filepath = ObjectProperty(None)
#     def __init__(self,**kwargs):
#         super().__init__(**kwargs)


# class ImportScreen(Screen):
#     filepath = ObjectProperty(None)
#     def __init__(self,**kwargs):
#         super().__init__(**kwargs)


# class MainScreen(Screen):
#     def __init__(self,**kwargs):
#         super().__init__(**kwargs)


###############################################################################
############################### GUI Application ###############################
###############################################################################

# class testApp(App):
class MotorsUI_App(App):
    title = "MVS"
    commandQueue = queue.Queue()

    def __init__(self,port=None,**kwargs):
        # super(testApp,self).__init__(**kwargs)
        super().__init__(**kwargs)
        self.serial_port = port
    
    def build(self):
        dt = 1/60

        self._mvs = MVSS()

        # petri_program = MVSS()
        # Clock.schedule_interval(petri_program.update,dt)
        return self._mvs

    def on_start(self):
        if self._mvs.start():
            self._comThread = threading.Thread(name="ComThread",target=self._mvs.run(),args=(self.commandQueue,))
            self._comThread.start()
            print("done")

###############################################################################
################################ Main Execution ###############################
###############################################################################

if __name__=="__main__":
    MotorsUI_App(port=arduino).run()
    # # testApp().run()

