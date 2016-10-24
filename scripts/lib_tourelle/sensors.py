#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 04 16:36:16 2014
Class to operate turret and sensors 
"""
__author__ = "Julien Favrichon, Jérémy Seyssaud"
__copyright__ = "Copyright 2014, CEA"
__license__ = ""
__version__ = "1.0.7" #24/02/2015
__status__ = "Prototype"

from abc import ABCMeta,abstractmethod

import cv2.cv as cv
import cv2
import random
import serial

from consts import *

class Sensor(object):
    """
    Abstract class for sensors    
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def acqOne(self): pass
    @abstractmethod
    def acqConti(self): pass
        
    def isEnable(self):
        return self.enable
    def isDisable(self):
        return not self.enable
    def setEnable(self):
        self.enable=True
    def setDisable(self):
        self.enable=False
    def getSensorType(self):
        return self.sensor_type
        
class Camera(Sensor):
    """
    Class for camera sensor
    """
    def __init__(self,cam_id=0,save_folder="/img", frame_width=0, frame_height=0):
        self.cam_id=cam_id
        self.save_folder=save_folder
        self.sensor_type=SENSOR_CAMERA
        self.frame_width=frame_width
        self.frame_height=frame_height
        
        #ajouter les parametres de la camera
        try:
            #self.capture = cv.CaptureFromCAM(self.cam_id)
            self.capture = cv2.VideoCapture(self.cam_id)
           
            if self.frame_width != 0 and self.frame_height != 0 :
                self.capture.set(cv.CV_CAP_PROP_FRAME_WIDTH,frame_width)
                self.capture.set(cv.CV_CAP_PROP_FRAME_HEIGHT,frame_height)
                
            print "self.capture.open()", self.capture.isOpened()
            print("Camera device connected")
                       
        except:
            print("Can't connect to camera device")
        
        else:
            self.enable=True
            print "Image Width",self.capture.get(cv.CV_CAP_PROP_FRAME_WIDTH)
            print "Image Heigh",self.capture.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
    
    def __str__(self):
        return "Sensor type Camera, device num {0}".format(self.cam_id)  
        
    def setDisable(self):
        super(Camera, self).setDisable()
        #Eco Energy : close CAM
        self.capture.release()
        #del (self.capture)
                
    def setEnable(self):
        #super(Camera, self).setEnabled()
        # open CAM device
        try:
            #self.capture = cv.CaptureFromCAM(self.cam_id)
            self.capture = cv2.VideoCapture(self.cam_id)
            if self.frame_width != 0 and self.frame_height != 0 :
                self.capture.set(cv.CV_CAP_PROP_FRAME_WIDTH,frame_width)
                self.capture.set(cv.CV_CAP_PROP_FRAME_HEIGHT,frame_height)
            print("Camera device connected")
        except:
            print("Can't connect to camera device")
            raise 
        else:
            self.enable=True
        
    def acqOne(self):
        #image = None        
        try:
            #try to resolve capture buffer issue            
            for i in range(5):
                empty_buff = self.capture.read()
                
            flag, im_array = self.capture.read()
            #image = cv.fromarray(im_array)
            #cv.SaveImage('output.jpeg', image)
            print("shot!")
           
            return im_array
        except:
            print("Can't connect to camera device")
            return False             
                             
    def acqConti(self, time_acq=0): #video
        '''
        Continius acquisition
        time : time of acquisition, 0 (default): infinite
        '''
        pass

    def __del__(self):
        #Release camera device
        self.capture.release()
        print("Cam class destructor")
          
class Telemeter(Sensor):
    '''
    Class for telemeter sensor
    '''
    def __init__(self,port,baudrate=115200,timeout=0.1):
        self.sensor_type=SENSOR_TELEMETER
        self.port=port
        self.baudrate=baudrate
        self.timeout=timeout

        try:
            self.tele_inter = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            print ("Connection to the telemeter OK")
        except:
            print ("Can't connect to the telemeter")
            #__del(self)__
        self.enable=True
        
    def __str__(self):
        return "Sensor type telemetre, device port {0}".format(self.port)

    def acqOne(self):
        self.tele_inter.flushInput()
        line = self.tele_inter.readline()
        line = line.replace(" ", "")
        line = line.replace("m", "")
        line = line.replace("\r\n", "")
        try:
            dist = float(line)
        except ValueError:
            print "ValueError: could not convert string to float:"
            dist = 0
        return dist
        
    def acqConti(self, time_acq=0):
        '''
        Continius acquisition
        time : time of acquisition, 0 (default): infinite
        '''
        pass

    def __del__(self):
        #Release telemeter device
        self.tele_inter.close()
        print("Telemeter class destructor")

class SensorTest(Sensor):
    '''
    Class sensor test
    Return random results
    '''
    def __init__(self,random_res=True,random_min=0,random_max=10):
        self.sensor_type=SENSOR_TEST
        self.enable=True
        self.random_res=random_res
        self.random_min=random_min
        self.random_max=random_max
            
    def __str__(self):
        return "Sensor type test"

    def acqOne(self):
        print "acqOne"
        if self.random_res:        
            return random.uniform(self.random_min, self.random_max)       # Random float x, 1.0 <= x < 10.00
        else:
            return "Test_acqOne"
        
    def acqConti(self, time_acq=0):
        '''
        Continius acquisition
        time : time of acquisition, 0 (default): infinite
        '''
        print "acqConti"
        return "Test_Conti"

    def __del__(self):
        #Release test
        print("Test class destructor")
        
if __name__ == "__main__": 
    print "Sensors Classes"