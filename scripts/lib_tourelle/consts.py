#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 04 16:36:16 2014
Constants for tourelle
"""
__author__ = "Julien Favrichon, Jérémy Seyssaud"
__copyright__ = "Copyright 2014, CEA"
__license__ = ""
__version__ = "1.0.7" #24/02/2015
__status__ = "Prototype"

# Constants
PAN_LIMIT_UP = 4095
PAN_LIMIT_DOWN = 0
TILT_LIMIT_UP = 1200
TILT_LIMIT_DOWN = 0
P_GAIN = 1
MOVING_SPEED = 20
MOVING_SPEED_RADS = 1

SENSOR_TEST = 0
SENSOR_CAMERA = 1
SENSOR_TELEMETER = 2


SENSOR_TELEMETER_L = 0.039 # hauteur / centre de rotation, en m
SENSOR_TELEMETER_l = 0.13 #longueur / centre de rotation, en m
SENSOR_TELEMETER_H = 0. #hauteur entre le sol ou le robot et le centre de rotation, en m


STITCHER = "./stitcher"

#SENSOR_TYPE = {SENSOR_CAMERA:'CAMERA',SENSOR_TELEMETER:'TELEMETER'}

if __name__ == "__main__":
    print "Constants"