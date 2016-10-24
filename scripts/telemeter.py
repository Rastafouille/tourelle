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



#!/usr/bin/env python
# -*- coding: utf-8 -*-


import serial
from math import sqrt, cos, sin
import rospy
from time import sleep
from threading import Lock
from sensor_msgs.msg import Range


class telemeter:
    def __init__(self):
        self.pub_telemeter=rospy.Publisher("/telemeter",Range,queue_size=5)
        
        self.dist=0

        #self.PORT=rospy.get_param("TELE_PORT",'/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DA00LE8S-if00-po')#'/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DA00L9O2-if00-port0') 
        self.PORT=rospy.get_param("TELE_PORT",'/dev/ttyUSB0')
        self.BAUDRATE=rospy.get_param("TELE_BAUDRATE",115200) 
        self.TIMEOUT=rospy.get_param("TELE_TIMEOUT",0.1) 
    
        self.d=Range()
        self.d.header.stamp = rospy.Time.now()
        self.d.header.frame_id="laser_link"
        self.d.field_of_view=0.1        
        self.d.radiation_type=rospy.get_param("TELE_RAD_TYPE",1)
        self.d.min_range=rospy.get_param("TELE_MIN_RANGE",0) 
        self.d.max_range=rospy.get_param("TELE_MAX_RANGE",40) 


        # connexion port serie       
        try:
            self.tele_inter = serial.Serial(port=self.PORT, baudrate=self.BAUDRATE, timeout=self.TIMEOUT)
            sleep(1)
            rospy.loginfo("Connection to the telemeter OK")
        except:
           rospy.loginfo("Can't connect to the telemeter")
            #__del(self)__
        self.enable=True

    def recup_trame(self):
        try :
            self.tele_inter.flushInput()
            line = self.tele_inter.readline()
            line = line.replace(" ", "")
            line = line.replace("m", "")
            line = line.replace("\r\n", "")
            #print line
            try:
                self.dist = float(line)
                #print self.dist
            except ValueError:
                    rospy.loginfo("Telemeter ValueError: could not convert string to float:")
                    self.dist = 0
        except :
            rospy.loginfo("recup trame Telemeter en rade !")

    def __del__(self):
        #Release telemeter device
        self.tele_inter.close()
        print("Telemeter class destructor")
            
            
    def pub_tel(self):
        self.d.range=self.dist
        self.d.header.stamp = rospy.Time.now()
        self.pub_telemeter.publish(self.d)
            
    def __delete__(self):
        self.tele_inter.close()
        print "Connexion fermee"
        
   
if __name__=="__main__":

    rospy.loginfo("Starting Telemeter")
    rospy.init_node("telemeter_node")
    my_telemeter=telemeter()
    rate=rospy.Rate(20)

    while not rospy.is_shutdown():
              my_telemeter.recup_trame()
              my_telemeter.pub_tel()
              rate.sleep()

    rospy.loginfo("Telemeter Node terminated")
    rospy.delete_param('~TELE_BAUDRATE')
    rospy.delete_param('~TELE_PORT')
    rospy.delete_param('~TELE_TIMEOUT')
    rospy.delete_param('~TELE_RAD_TYPE')
    rospy.delete_param('~TELE_MIN_RANGE')
    rospy.delete_param('~TELE_MAX_RANGE')
    rospy.delete_param('~TELE_TYPE')
    rospy.delete_param('~SENSOR_TELEMETER_H')
    rospy.delete_param('~SENSOR_TELEMETER_L')
    rospy.delete_param('~SENSOR_TELEMETER_l')    
    
    
    
    my_telemeter.tele_inter.close()
    rospy.loginfo("Connexion Telemeter fermee")
