#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 04 16:36:16 2014
Class to operate dose rate sensor 
"""
__author__ = "Julien Favrichon, Jérémy Seyssaud"
__copyright__ = "Copyright 2014, CEA"
__license__ = ""
__version__ = "1.0.7" #24/02/2015
__status__ = "Prototype"

import serial
import struct
import rospy
from time import sleep
from tourelle_ros.msg import Dose_rate
import random


class dose_rate_sensor:
    def __init__(self):
        self.pub_dose_rate=rospy.Publisher("/dose_rate",Dose_rate,queue_size=10)
        self.d=Dose_rate()
#        self.d.header.frame_id=dose_rate_sensor_link

        self.PORT=rospy.get_param("DRS_PORT",'/dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FTYK2VPH-if00-port0')
        self.BAUDRATE=rospy.get_param("DRS_BAUDRATE",38400) 
        self.TIMEOUT=rospy.get_param("DRS_TIMEOUT",0.1) 
        self.d.min_range=rospy.get_param("DRS_MIN_RANGE",0.0) 
        self.d.max_range=rospy.get_param("DRS_MAX_RANGE",2000.0)
        self.TRAME_COUPS= bytearray([0x01,0x04,0x75,0x32,0x00,0x02,0xCA,0x08])        
        self.TRAME_DDD= bytearray([0x01,0x04,0x75,0x30,0x00,0x02,0x6B,0xC8])

          # connexion port serie       
        try:
            self.IMS_com = serial.Serial(port=self.PORT, baudrate=self.BAUDRATE, timeout=self.TIMEOUT)
            sleep(1)
            rospy.loginfo("Connection to the IMS probe OK")
        except:
           rospy.loginfo("Can't connect to the IMS probe")
        
    def recup_DdD(self):    
        try:
            self.IMS_com.write(self.TRAME_DDD)
            self.IMS_com.flushOutput()
            self.IMS_com.flushInput()
            line = self.IMS_com.readline()
            line = line.encode('hex')
            line=struct.unpack('f',struct.pack('i',int(line[6:14],16)))
            self.d.dose_rate=line[0]
            self.d.header.stamp = rospy.Time.now()
            self.pub_dose_rate.publish(self.d)   
            #print line[0]            
        except:
            rospy.loginfo("recup trame DdD en rade !")
        
    def random_ddd(self):
            self.d.dose_rate=random.random()*self.d.max_range
            self.d.header.stamp = rospy.Time.now()
            self.pub_dose_rate.publish(self.d)
            
    def __del__(self):
        print("Dose_rate_sensor class destructor")   

    def __delete__(self):
        self.IMS_com.close()
        print "Connexion IMS probe fermee"
        
   
if __name__=="__main__":

    print "Starting Dose Rate Sensor"
    rospy.init_node("dose_rate_node")
    my_dose_rate_sensor=dose_rate_sensor()
    rate=rospy.Rate(2)

    while not rospy.is_shutdown():
        #my_dose_rate_sensor.recup_DdD()
        my_dose_rate_sensor.random_ddd()            
        rate.sleep()

    rospy.loginfo("Dose Rate Sensor Node terminated")
    rospy.delete_param("~DRS_BAUDRATE")
    rospy.delete_param("~DRS_PORT")
    rospy.delete_param("~DRS_TIMEOUT")
    rospy.delete_param("~DRS_MIN_RANGE")
    rospy.delete_param("~DRS_MAX_RANGE")
    rospy.delete_param("~DRS_H")
    rospy.delete_param("~DRS_L")
    rospy.delete_param("~DRS_l")    
    #my_telemeter.tele_inter.close()
    rospy.loginfo("Connexion Dose Rate Sensor fermee")
