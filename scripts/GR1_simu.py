#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#
# Copyright (C) 2014  Lionel Bergeret
#
# ----------------------------------------------------------------
# The contents of this file are distributed under the CC0 license.
# See http://creativecommons.org/publicdomain/zero/1.0/
# ----------------------------------------------------------------
#import hid
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from tourelle_ros.msg import GR1_msg
#from tourelle_ros.srv import spectro_clear
from std_srvs.srv import Empty,EmptyResponse
import math
import random

class GR1():
    def __init__(self):
        self.USB_VENDOR_ID=rospy.get_param("/tourelle/GR1_node/GR1_VENDOR_ID",0x2a5a) 
        self.USB_PRODUCT_ID=rospy.get_param("/tourelle/GR1_node/GR1_PRODUCT_ID",0x0050) 
        self.COUNTRATE_INTERVAL=rospy.get_param("/tourelle/GR1_node/GR1_COUNTRATE_INTERVAL",1) #en secondes
        self.GR1_K=rospy.get_param("/tourelle/GR1_node/GR1_K",1.0) #etalonnage H*10 en microSv/h / cps 
        self.GR1_A=rospy.get_param("/tourelle/GR1_node/GR1_A",1) #etalonage en energie energie(kev) = A * canal + B
        self.GR1_B=rospy.get_param("/tourelle/GR1_node/GR1_B",1) #etalonage en energie energie(kev) = A * canal + B
        print "Creation du service spectro clear"
        self.service=rospy.Service("/tourelle/GR1_node/clear",Empty,self.cb_service_clear)    #déclaration du service        
        
        #GR10        
#        self.USB_VENDOR_ID = 0x04d8
#        self.USB_PRODUCT_ID = 0x0000
#        self.COUNTRATE_INTERVAL = 20

        #GR05
        self.USB_VENDOR_ID = 0x2a5a
        self.USB_PRODUCT_ID = 0x0050
#        self.COUNTRATE_INTERVAL = 1
        
        self.pub_GR1 = rospy.Publisher('/tourelle/GR1', GR1_msg, queue_size=10) 
        self.d=GR1_msg()
        self.d.header.frame_id='collimator_link'   
        self.d.header.stamp = rospy.Time.now()
        
        self.ihm = False
        
        
        usbPathList = []
        """
        # Enumarate HID devices
        for d in hid.enumerate(0, 0):
            keys = d.keys()
            keys.sort()
        if d["vendor_id"] == self.USB_VENDOR_ID:
           usbPathList.append(d["path"])        
        self.devicePath = usbPathList
        print "Available Kromek devices =", self.devicePath
        print self.USB_PRODUCT_ID, self.USB_VENDOR_ID
 
        self.hidDevice = hid.device(self.USB_VENDOR_ID,self.USB_PRODUCT_ID)
        """
        # Initialize counters        
        self.counts=np.zeros(4096) 
        self.totalcounter = 0 # keep track of total counts since start
        self.temps_ref=rospy.Time.now()
        if self.ihm:
            plt.ion()
            plt.figure()
            self.x=np.linspace(0,4095,4096)
            self.affichage()
        
        self.time_begin = time.time()  # for simu
        
    def Process(self):
        
        #remise a zero counter
        #self.counts=np.zeros(4096) 
        self.totalcounter = 0

        # Start timers
        start_time = time.time()
        while (time.time() - start_time) < self.COUNTRATE_INTERVAL:
            d = self.hidDevice.read(62)# timeout_ms = 50)
            if d:
                self.totalcounter += 1
                #print("impact")
                channel = (d[1]*256+d[2])/16 # ((d[1] << 8 | d[2]) >> 4) = 12bit channel
                self.counts[channel] += 1
            time.sleep(0.0001) # force yield for other threads

        self.d.spectro=self.counts
        deltat=rospy.Time.now()-self.d.header.stamp
        self.d.count_rate=float(self.totalcounter)/deltat.to_sec()
        self.d.dose_rate=(self.totalcounter*self.GR1_K)
        self.d.header.stamp = rospy.Time.now()
        self.pub_GR1.publish(self.d)
        plt.clf()         
        if self.ihm : self.affichage()
        
    def process_simu(self):
        func_sin = lambda x: math.fabs(int(300 * math.sin(2 * math.pi * x / 60))) #periode 60s
        
        self.totalcounter = 0
        
        # Start timers
        start_time = time.time()
        """
        while (time.time() - start_time) < self.COUNTRATE_INTERVAL:
            d = func_sin(time.time()-self.time_begin)
            print "func_sin",d
            self.totalcounter += d
            #print("impact")
            channel = random.randint(0, 4095) # ((d[1] << 8 | d[2]) >> 4) = 12bit channel
            self.counts[channel] += 1
            time.sleep(0.01) # force yield for other threads
        """
        d = func_sin(time.time()-self.time_begin)
        #print("impact")
        self.totalcounter += d
        channel = random.randint(0, 4095) # ((d[1] << 8 | d[2]) >> 4) = 12bit channel
        self.counts[channel] += 1
        time.sleep(self.COUNTRATE_INTERVAL)     
        
        self.d.spectro = self.counts
        deltat = rospy.Time.now() - self.d.header.stamp
        self.d.count_rate = float(self.totalcounter)/deltat.to_sec()
        self.d.dose_rate = float(self.d.count_rate*self.GR1_K)
        print "count_rate = ",self.d.count_rate
        print "dose_rate = ",self.d.dose_rate
        self.d.header.stamp = rospy.Time.now()
        self.pub_GR1.publish(self.d)
        plt.clf()         
        if self.ihm : self.affichage()
        
    def cb_service_clear(self,req):
        print "Spectro CLEAR SERVICE"
        self.counts=np.zeros(4096) 
        self.totalcounter = 0        
        self.temps_acq=0
        self.temps_ref=rospy.Time.now()
        self.time_begin = time.time()  # for simu
        
        return EmptyResponse()
        
    def affichage(self):
        self.temps_acq=rospy.Time.now()-self.temps_ref
        plt.clf()         
        plt.ylabel('spectre de la sonde GR1')
        plt.xlabel("channels")
        plt.title('Comptage: %.2f cps, DeD H*10: %.2f Sv/h, duree:%.1f s'%(self.d.count_rate,self.d.dose_rate,self.temps_acq.to_sec()))
        plt.plot(self.x,[(self.counts[i] ) for i in range(4096)],marker='v',color='r')                 
        plt.show()
        plt.pause(0.0001)
        
    def __del__(self):
        self.hidDevice.close()
        print "Connexion GR1 fermee"
    

if __name__ == '__main__':

    rospy.loginfo("Starting GR1")
    rospy.init_node("GR1_node")
    rate=rospy.Rate(10)    
    GR1 = GR1()
    #rospy.wait_for_service("/tourelle/GR1_node/clear")
    
    while not rospy.is_shutdown():
        #GR1.Process()
        GR1.process_simu()
        rate.sleep()


    rospy.loginfo("GR1 Node terminated")
    """
    rospy.delete_param('~GR1_VENDOR_ID')
    rospy.delete_param('~GR1_PRODUCT_ID')
    rospy.delete_param('~GR1_COUNTRATE_INTERVAL')
    rospy.delete_param('~GR1_K')
    rospy.delete_param('~GR1_B')
    rospy.delete_param('~GR1_A')
    """


  
