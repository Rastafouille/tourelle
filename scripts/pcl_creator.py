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

from math import sqrt, cos, sin
import rospy
from time import sleep
from threading import Lock
from sensor_msgs.msg import PointCloud, Range
from geometry_msgs.msg import Point32, PointStamped
import tf

class pcl_creator:
    def __init__(self):
        self.sub_telemeter=rospy.Subscriber("/tourelle/telemeter",Range, self.cb_telemeter)
        self.pub_pcl=rospy.Publisher("/tourelle/pcl_acqui",PointCloud,queue_size=10)
        
        self.listener = tf.TransformListener()
           
        
        self.tele_point=PointStamped()
        self.tele_point.header.stamp=rospy.Time.now()
        self.tele_point.header.frame_id='laser_link'
        self.tele_point.point.x=0
        self.tele_point.point.y=0
        self.tele_point.point.z=0

        self.base_point=PointStamped()
        self.tele_point.header.stamp=rospy.Time.now()
        self.base_point.header.frame_id='base_link'
        #self.base_point=self.listener.transformPoint("world",self.tele_point)
        
        self.pcl=PointCloud()
        self.pcl.header.stamp = rospy.Time.now()
        self.pcl.header.frame_id="base_link"
     
    def pcl_acqui(self):
        try :
            self.listener.waitForTransform("base_link", "laser_link", self.tele_point.header.stamp, rospy.Duration(0.5))
            self.base_point=self.listener.transformPoint("base_link",self.tele_point)
            self.base_point.header.stamp=self.tele_point.header.stamp
            self.pcl.points.append(Point32(self.base_point.point.x,self.base_point.point.y,self.base_point.point.z))
            self.pcl.header.stamp = self.tele_point.header.stamp
            self.pub_pcl.publish(self.pcl)
            print self.pcl
        except:
            print "ERROR pcl acquisition"
            pass
        
    def __del__(self):
        #Release telemeter device
        print("Pcl Creator class destructor")
            
            
    def cb_telemeter(self,msg):
        if (msg.range>(self.tele_point.point.x+0.01)) | (msg.range<(self.tele_point.point.x-0.01)) :
            if msg.range!=0:            
                self.tele_point.header.stamp=msg.header.stamp
                self.tele_point.point.x=msg.range
                self.pcl_acqui()
                print "OK"
            
    def __delete__(self):
        self.tele_inter.close()
        print "Connexion fermee"
        
   
if __name__=="__main__":


    print "Starting pcl_creator"
    rospy.init_node("pcl_creator")
    my_pcl_creator=pcl_creator()
    
    rate=rospy.Rate(20)
    
    rospy.spin()

    rospy.loginfo("my_pcl_creator Node terminated")
