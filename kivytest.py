#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 19 11:42:48 2017

@author: mac
"""

#from IPython import get_ipython
#get_ipython().magic('reset -sf')
from kivy.app import App

class TestApp(App):
#    def __init__(self):
#        App.__init__(self)
    def build(self):
        kv_directory = '/Users/mac/Documents/MVSS'
    def change_text(self):
        return "New Text"

if __name__ == '__main__':
    TestApp().run()