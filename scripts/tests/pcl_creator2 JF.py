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

from math import sqrt, cos, sin
import rospy
from time import sleep
from threading import Lock
from sensor_msgs.msg import PointCloud2, Range, Image, PointField
from geometry_msgs.msg import Point32, PointStamped
from tourelle_ros.msg import Dose_rate
import numpy as np
import tf
import random
#import pcl_ros
#from pointcloud.msg import PointXYZRGB

class pcl2_creator:
    def __init__(self):
 
        self.telemeter_point = PointStamped()
        self.telemeter_point.header.stamp = rospy.Time.now()
        self.telemeter_point.header.frame_id ='laser_link'
        self.telemeter_point.point.x = 0
        self.telemeter_point.point.y = 0
        self.telemeter_point.point.z = 0
        
        self.essai_point = PointStamped()
        self.essai_point.header.stamp = rospy.Time.now()
        self.essai_point.header.frame_id ='base_link'
        self.essai_point.point.x = 0
        self.essai_point.point.y = 0
        self.essai_point.point.z = 0

        self.base_point = PointStamped()
        self.base_point.header.stamp = rospy.Time.now()
        self.base_point.header.frame_id = 'base_link'
            
        self.pcl2 = PointCloud2()
        self.pcl2.header.stamp = rospy.Time.now()
        self.pcl2.header.frame_id = "base_link"
        
        
        #self.sub_telemeter = rospy.Subscriber("/tourelle/telemeter",Range, self.cb_telemeter)
        
        #self.sub_dose_rate = rospy.Subscriber("/dose_rate",Dose_rate, self.cb_dose_rate) 
        #self.sub_usb_cam = rospy.Subscriber("/tourelle/usb_cam/image_raw",Image, self.cb_usb_cam)
        
        self.tf_listener = tf.TransformListener()
                
        self.pub_pcl2 = rospy.Publisher("/pcl2",PointCloud2,queue_size=10)    
        self.pcl2.fields=[
            PointField('x',0,PointField.FLOAT32, 1),
            PointField('y',4,PointField.FLOAT32, 1),
            PointField('z',8,PointField.FLOAT32, 1),
            PointField('rgb',12,PointField.UINT32, 1)]
#        
#        self.a
        #self.pcl2.data= str([[0,0.1,0.2],1,2,3,6])
#        
    def __del__(self):
        #Release telemeter device
        print("Pcl2 Creator class destructor")
            
    def xyzrgb2pc(self, xyz, bgr, frame_id='', use_time_now=True):
    
        xyz = np.asarray(xyz)
        bgr = np.asarray(bgr)
    
        assert xyz.shape == bgr.shape
        if xyz.ndim == 2:
            xyz = xyz[None, :, :]
            bgr = bgr[None, :, :]
    
        height = xyz.shape[0]
        try:
            width = xyz.shape[1]
        except:
            width = 1
            
        arr = np.empty((height, width, 8), dtype='float32')
        arr[:, :, 0:3] = xyz
        bgr1 = np.empty((height, width, 4), dtype='uint8')
        bgr1[:, :, 0:3] = bgr
        arr[:, :, 4] = bgr1.view(dtype='float32').reshape(height, width)
        data = arr.tostring()
        msg = PointCloud2()
        msg.data = data
        msg.header.frame_id = frame_id
        msg.fields = [PointField(name='x',offset=0,datatype=7,count=1),
                      PointField(name='y',offset=4,datatype=7,count=1),
                      PointField(name='z',offset=8,datatype=7,count=1),
                      PointField(name='rgb',offset=16,datatype=7,count=1)]
        msg.is_dense = False
        msg.width = width
        msg.height = height
        if use_time_now:
            msg.header.stamp = rospy.Time.now()
    
        msg.point_step = 32
        msg.row_step = 32 * width
        msg.is_bigendian = False
    
        return msg
    """
    def xyz_rgb_array_to_pointcloud2(self,points,colors, stamp=None, frame_id=None):
        '''
        Create a sensor_msgs.PointCloud2 from an array
        of points and rgba value
        '''
        
        xyz = np.asarray(points,np.float32)
        bgr = np.asarray(colors,np.uint32)
    
        assert xyz.shape == bgr.shape
        if xyz.ndim ==2:
            xyz = xyz[None,:,:]
            bgr = bgr[None,:,:]
        
        print "xyz",xyz
        print "bgr",bgr
        
        #height= xyz.shape[0]
        #width = xyz.shape[1]
        width = 3
        height = 1
        
        arr = np.empty((height,width,8),dtype='float32')
        arr[:,:,0:3] = xyz
        #arr[:,:,0] = xyz
        print "arrxyz",arr
        bgr1 = np.empty((height,width,4),dtype='uint8')
        bgr1[:,:,0:3] = bgr   
        print "bgr1",bgr1
        arr[:,:,4] = bgr1.view(dtype='float32').reshape(height, width)
        print "arr",arr
        data = arr.tostring()
        msg = PointCloud2()
        msg.data = data
        msg.header.frame_id = frame_id
        msg.fields = [PointField(name='x',offset=0,datatype=7,count=1),
                      PointField(name='y',offset=4,datatype=7,count=1),
                      PointField(name='z',offset=8,datatype=7,count=1),
                      PointField(name='rgb',offset=16,datatype=7,count=1)]
        msg.is_dense = False
        msg.width=width
        msg.height=height
        
        if stamp:
            msg.header.stamp = stamp
        else :
            msg.header.stamp = rospy.Time.now()
        if frame_id:
            msg.header.frame_id = frame_id
       
        #msg.point_step = 32
        #msg.row_step = 32 * width
        msg.point_step = 16
        msg.row_step = 32
        msg.is_bigendian = False        
        
        return msg
    """    

        
    def startAcqui(self):
        #print self.pcl2
      
        #Valid channel names: rgb (1 channel), r, g, b (3 channel)
        #RGB only affects the color of the point.
        #There are two ways to specify RGB:
        #    3 channels, named "r", "g", and "b", with floating point values between 0 and 1.
        #    1 channel, with the float in the channel reinterpreted as 3 single-byte values with ranges from 0 to 255. 0xff0000 is red, 0xff00 is green, 0xff is blue.
        #        In C++, int rgb = 0xff0000; float float_rgb = *reinterpret_cast<float*>(&rgb); 
        #        In Python,  float_rgb = struct.unpack('f', struct.pack('i', 0xff0000))[0]
      
        #pts = [0.,1.,2.]
        pts = np.array([1.,2.,3.])
        #colors=[0,0,0]
        colors = np.array([0,0,0])
        while not rospy.is_shutdown():      
            try :
                """
                #Recuperation des points du télémètre
                self.tf_listener.waitForTransform("base_link", "laser_link", self.telemeter_point.header.stamp, rospy.Duration(0.5))
                self.base_point = self.tf_listener.transformPoint("base_link",self.telemeter_point)
                #print ('tele_point.header.stamp = ',self.telemeter_point.header.stamp,'/ base_point.header.stamp= ',self.base_point.header.stamp)            
                self.base_point.header.stamp = self.telemeter_point.header.stamp
                """                         
                
                #print "self.pcl2",self.pcl2
                #self.pub_pcl2.publish(self.pcl2)
                                
                """                
                self.pcl.points.append(Point32(self.base_point.point.x,self.base_point.point.y,self.base_point.point.z))
                self.pcl.header.stamp = self.tele_point.header.stamp
                self.pub_pcl.publish(self.pcl)
                """
                sleep(0.01)
            
            except:
                print "ERROR pcl acquisition"
                pass
            
            #self.pcl2.data=str([[x,y,z],r,g,b,d])
            #self.pcl2.header.stamp = self.telemeter_point.header.stamp
            #pts = np.random.random_integers(10, size=(1.,3.))
            x = random.randint(0, 5)
            y = random.randint(0, 5)
            z = random.randint(0, 5)
            #pts=[float(x),1.,2.]
            pts_new = np.array([float(x),float(y),float(z)])
            pts = np.vstack((pts,pts_new))
            print "pts",pts
            #colors = np.random.random_integers(255, size=(1.,3.))
            r = random.randint(0, 255)
            g = random.randint(0, 255)
            b = random.randint(0, 255)
            #colors=[r,g,b]
            #colors = [r,g,b]
            colors_new = np.array([r,g,b])
            colors = np.vstack((colors,colors_new))
            #mon_pcl = self.xyz_rgb_array_to_pointcloud2(pts, colors,stamp=rospy.Time.now(), frame_id="essai_point")
            mon_pcl = self.xyzrgb2pc(xyz=pts,bgr=colors,frame_id='base_link',use_time_now=True)
            #print mon_pcl
            self.pub_pcl2.publish(mon_pcl)
            #self.pub_pcl2.publish(self.pcl2)
        
    def cb_telemeter(self,msg):
        """
        if (msg.range>(self.telemeter_point.point.x+0.01)) | (msg.range<(self.telemeter_point.point.x-0.01)):
            self.telemeter_point.header.stamp=rospy.Time.now()#msg.header.stamp
            self.telemeter_point.point.x=msg.range
            sleep (0.01)
        """
        if (msg.range>(self.telemeter_point.point.x+0.01)) | (msg.range<(self.telemeter_point.point.x-0.01)) :
            if msg.range!=0:            
                self.telemeter_point.header.stamp=msg.header.stamp
                self.telemeter_point.point.x=msg.range
                #print msg.range
                 
    def cb_dose_rate(self,msg):
        sleep (0.01)

    def cb_usb_cam(self,msg):
        sleep (0.01)

           
    def __delete__(self):
        self.tele_inter.close()
        print "Connexion fermee"
        
   
if __name__=="__main__":


    print "Starting pcl2 creator node"
    rospy.init_node("pcl_creator_node")
    my_pcl2_creator=pcl2_creator()
    rate=rospy.Rate(50)
    sleep(1)
    my_pcl2_creator.startAcqui()

    #rospy.spin()

    rospy.loginfo("my_pcl_creator Node terminated")
