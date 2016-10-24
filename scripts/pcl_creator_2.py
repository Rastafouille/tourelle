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

import rospy
from time import sleep
import rospkg
import os
from sensor_msgs.msg import PointCloud2, Range, PointField
from geometry_msgs.msg import PointStamped
from tourelle_ros.msg import GR1_msg
import numpy as np
import tf
from std_srvs.srv import Empty,EmptyResponse
from tourelle_ros.srv import SetScale,SetScaleRequest,SetScaleResponse


class pcl2_creator:
    def __init__(self):
        self.listener = tf.TransformListener() 
 
        self.telemeter_point = PointStamped()
        self.telemeter_point.header.stamp = rospy.Time.now()
        self.telemeter_point.header.frame_id ='laser_link'
        self.telemeter_point.point.x = 0
        self.telemeter_point.point.y = 0
        self.telemeter_point.point.z = 0

        self.base_point = PointStamped()
        self.base_point.header.stamp = rospy.Time.now()
        self.base_point.header.frame_id = 'base_link'
        """
        self.pcl2 = PointCloud2()
        self.pcl2.header.stamp = rospy.Time.now()
        self.pcl2.header.frame_id = "base_link" 
        self.pcl2.fields=[
            PointField('x',0,PointField.FLOAT32, 1),
            PointField('y',4,PointField.FLOAT32, 1),
            PointField('z',8,PointField.FLOAT32, 1),
            PointField('rgb',12,PointField.UINT32, 1)]
        """
        self.CPS_MAX=rospy.get_param("/tourelle/GR1_node/GR1_COUNTRATE_MAX",10.0)
        self.GR1_K=rospy.get_param("/tourelle/GR1_node/GR1_K",1.0) #etalonnage H*10 en microSv/h / cps
        self.DOSE_MAX = self.CPS_MAX*self.GR1_K
        print "dose max:",self.DOSE_MAX
        self.DOSE_MIN = 0.0        
        
        self.pclCLEAR=True
        self.process_scale = False
        
        self.count_rate=0
        self.ded = np.array([0])        
        self.pts = np.array([0,0,0])
        self.colors = np.array([0,0,0])
        
 # LUT for DED
        rp = rospkg.RosPack()
        color_file = os.path.join(rp.get_path('tourelle_ros'), 'scripts', 'colormap')
        lut_file = np.loadtxt(color_file,dtype=np.uint8) #RGB
        self.lut_bgr = np.fliplr(lut_file) #BGR     
        
        self.pub_pcl2 = rospy.Publisher("/tourelle/pcl2",PointCloud2,queue_size=10)
        self.sub_telemeter=rospy.Subscriber("/tourelle/telemeter",Range, self.cb_telemeter)
        self.sub_dose_rate = rospy.Subscriber("/tourelle/GR1",GR1_msg, self.cb_GR1) 
        
        print "Creation du service pcl2 clear"
        self.service=rospy.Service("/tourelle/pcl2_creator_node/clear",Empty,self.cb_service_clear)    #déclaration du service        
        print "Creation du service pcl2 pcd_save"
        self.service=rospy.Service("/tourelle/pcl2_creator_node/pcd_save",Empty,self.cb_service_pcd_save)
        print "Creation du service pcl2 set_scale"
        self.service_set_scale = rospy.Service("/tourelle/pcl2_creator_node/set_scale",SetScale,self.cb_service_set_scale)

    def __del__(self):
        #Release telemeter device
        print("Pcl2 Creator class destructor")
            

    def cb_service_clear(self,req):
        print "PCL2 CLEAR SERVICE"
        self.count_rate=0
        self.pclCLEAR=True
        return EmptyResponse()

    def cb_service_pcd_save(self,req):
        res_file=open("result_telemeter.pcd","w")
        buf = "# .PCD v.7 - Point Cloud Data file format\n"
        buf +="VERSION .7\n"
        buf +="FIELDS x y z rgb\n"
        buf +="SIZE 4 4 4 4\n"
        buf +="TYPE F F F I\n"
        buf +="COUNT 1 1 1 1\n"
        buf +="WIDTH "+str(len(self.pts))+"\n"
        buf +="HEIGHT 1\n"
        buf +="VIEWPOINT 0 0 0 1 0 0 0\n"
        buf +="POINTS "+str(len(self.pts))+"\n"
        buf +="DATA ascii\n"
        print buf
        print "ok"        
        for i in range(0,len(self.pts)-1):
            print str(self.pcl2.data[i])+" "+str(self.pcl2.data[i+1])+" "+str(self.pcl2.data[i+2])+" "+str(self.pcl2.data[i+3])+"\n"           
            buf += str(self.pcl2.data[i])+" "+str(self.pcl2.data[i+1])+" "+str(self.pcl2.data[i+2])+" "+str(self.pcl2.data[i+3])+"\n"
        res_file.write(buf)
        res_file.close()
        print buf
        print "PCD SAVE SERVICE"
        return EmptyResponse()

    def pcl2_acqui(self):
#        try :
            self.listener.waitForTransform("base_link", "laser_link", self.telemeter_point.header.stamp, rospy.Duration(1))
            self.base_point=self.listener.transformPoint("base_link",self.telemeter_point)
            self.base_point.header.stamp=self.telemeter_point.header.stamp
            pts_new = np.array([float(self.base_point.point.x),float(self.base_point.point.y),float(self.base_point.point.z)])
            ded_new = (self.count_rate*self.GR1_K)
            color_canal = int((ded_new - self.DOSE_MIN) * 255.0 / (self.DOSE_MAX - self.DOSE_MIN))
            if color_canal < 0 : color_canal = 0       
            if color_canal > 255 : color_canal = 255 
            r = self.lut_bgr  [color_canal][2]
            g = self.lut_bgr  [color_canal][1]
            b = self.lut_bgr  [color_canal][0]
            colors_new = np.array([b,g,r])
           
            if self.process_scale:
                for i in range(0,len(self.colors)-1):
                    color_canal = int((self.ded[i] - self.DOSE_MIN) * 255.0 / (self.DOSE_MAX - self.DOSE_MIN))
                    if color_canal < 0 : color_canal = 0
                    if color_canal > 255 : color_canal = 255                    
                    r = self.lut_bgr  [color_canal][2]
                    g = self.lut_bgr  [color_canal][1]
                    b = self.lut_bgr  [color_canal][0]
                    colors_new = np.array([b,g,r])
                    self.colors[i]=colors_new      
                self.process_scale = False
            
            if self.pclCLEAR:
                self.ded = ded_new                
                self.pts = pts_new
                self.colors = colors_new
                self.pclCLEAR=False	                
            else :     
                self.ded = np.vstack((self.ded,ded_new))                
                self.pts = np.vstack((self.pts,pts_new))
                self.colors = np.vstack((self.colors,colors_new))
            
            mon_pcl2 = self.xyzrgb2pc(xyz=self.pts,bgr=self.colors,frame_id='base_link',time2use=self.telemeter_point.header.stamp)
            self.pub_pcl2.publish(mon_pcl2)            
            #self.pcl2.header.stamp = self.telemeter_point.header.stamp
            #self.pub_pcl2.publish(self.pcl2)
            #print self.pcl2
#        except:
#            print "ERROR pcl acquisition"
#            pass        
        
        
    def cb_telemeter(self,msg):
        if (msg.range>(self.telemeter_point.point.x+0.01)) | (msg.range<(self.telemeter_point.point.x-0.01)) :
            if msg.range!=0:            
                self.telemeter_point.header.stamp=msg.header.stamp
                self.telemeter_point.point.x=msg.range
                self.pcl2_acqui()
                 
    def cb_GR1(self,msg):
        self.count_rate=msg.count_rate

    def xyzrgb2pc(self, xyz, bgr, frame_id='', time2use=0):    
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
        msg.header.stamp = time2use
        msg.point_step = 32
        msg.row_step = 32 * width
        msg.is_bigendian = False
    
        return msg
               
    def cb_service_set_scale(self,msg):
        self.DOSE_MAX = msg.scale_max * 1000 #(mSv/h -> uSv/h)
        self.DOSE_MIN = msg.scale_min * 1000 #(mSv/h -> uSv/h)
        self.process_scale = True
        #process sclae?
        return SetScaleResponse()               
               
               
    def __delete__(self):
        self.tele_inter.close()
        print "Connexion fermee"
        
   
if __name__=="__main__":

    sleep(1) #delais pour le chargement des param...
    print "Starting pcl2 creator node"
    rospy.init_node("pcl2_creator_node")
    my_pcl2_creator=pcl2_creator()
    rate=rospy.Rate(50)
    sleep(1)
    rospy.spin()

    rospy.loginfo("my_pcl_creator Node terminated")



