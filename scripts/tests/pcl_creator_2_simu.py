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
from std_msgs.msg import Float32, Header, Bool
from sensor_msgs.msg import PointCloud2, Range, PointField, PointCloud, ChannelFloat32
from geometry_msgs.msg import PointStamped, Point32
from tourelle_ros.msg import GR1_msg,Dose_PCL
import numpy as np
import tf
import timeit
#import sensor_msgs.point_cloud2 as pcl2
from dynamixel_msgs.msg import JointState

from std_srvs.srv import Empty,EmptyResponse
#import pcl_ros
#import tf2_sensor_msgs


class pcl2_creator:
    def __init__(self):
        self.pub_pcl2 = rospy.Publisher("/tourelle/pcl2",PointCloud2,queue_size=3)
        
        #self.pub_pcl2_dose = rospy.Publisher("/tourelle/pcl2_dose",PointCloud2,queue_size=10)
        self.pub_pcl_dose = rospy.Publisher("/tourelle/pcl_dose",Dose_PCL,queue_size=3)
        self.pub_pcl_camera = rospy.Publisher("/tourelle/pcl_camera",PointCloud,queue_size=3)
        
        self.sub_telemeter=rospy.Subscriber("/tourelle/telemeter",Range, self.cb_telemeter)
        self.sub_dose_rate = rospy.Subscriber("/tourelle/GR1",GR1_msg, self.cb_GR1) 
        
        self.sub_acq_cmd = rospy.Subscriber("/cmd_acq",Bool, self.cb_cmd_acq) 
        
        print "Creation du service pcl2 clear"
        self.service=rospy.Service("/tourelle/pcl_creator_node/clear",Empty,self.cb_service_clear)    #déclaration du service        
        print "Creation du service pcl2 pcd save"
        self.service=rospy.Service("/tourelle/pcl_creator_node/pcd_save",Empty,self.cb_service_pcd_save)
        
        #Subscriber
        self.sub_tilt=rospy.Subscriber("/tourelle/j_ACC_PLT_position_controller/state",JointState, self.cb_tilt)
        self.sub_pan=rospy.Subscriber("/tourelle/j_BASE_ACC_position_controller/state",JointState, self.cb_pan)        
        
        self.listener = tf.TransformListener() 
        
        self.listener_camera = tf.TransformListener()
 
        self.telemeter_point = PointStamped()
        self.telemeter_point.header.stamp = rospy.Time.now()
        self.telemeter_point.header.frame_id ='laser_link'
        self.telemeter_point.point.x = 0
        self.telemeter_point.point.y = 0
        self.telemeter_point.point.z = 0
        
        self.collimator_point = PointStamped()
        self.collimator_point.header.stamp = rospy.Time.now()
        self.collimator_point.header.frame_id ='collimator_link'
        self.collimator_point.point.x = 0
        self.collimator_point.point.y = 0
        self.collimator_point.point.z = 0

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
        self.ddd_min_range=rospy.get_param("DRS_MIN_RANGE",0.0) 
        self.ddd_max_range=rospy.get_param("DRS_MAX_RANGE",5.0)
        self.cps_max = rospy.get_param("/tourelle/GR1_NODE/GR1_COUNTRATE_MAX",5)
        
        self.pcl = PointCloud()
        self.camera_pcl = PointCloud()
        
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        #self.pcl.header.stamp = rospy.Time.now()
        #self.pcl.header.frame_id = "base_link"
        self.pcl.header = header
        channel_dose = ChannelFloat32('dose', [])
        self.pcl.channels.append(channel_dose)
        self.doses = []
        
        self.ddd=0
        self.pts = np.array([0,0,0])
        self.colors = np.array([0,0,0])
        
        # Flag pour voir si la tourelle est ou a été en mouvement
        self.pan_velocity_flag = True
        self.tilt_velocity_flag = True
        
        self.acq_seq = True
        self.process_cpl_flag = False

    def __del__(self):
        #Release telemeter device
        print("Pcl2 Creator class destructor")
        
    def cb_cmd_acq(self,msg):
        print "cb_cmd_acq:",msg.data
        self.acq_seq = msg.data
        sleep(0.001)
            
    def cb_pan(self,msg):
        #float64 velocity    # current joint speed (in radians per second)
        if msg.velocity != 0.0:
            self.pan_velocity_flag = True
            self.process_cpl_flag = True
        sleep(0.001)
  
    def cb_tilt(self,msg):
        if msg.velocity != 0.0:
            self.tilt_velocity_flag = True
            self.process_cpl_flag = True
        sleep(0.001)
        
    def cb_service_clear(self,req):
        print "PCL2 CLEAR SERVICE"
        self.ddd=0
        self.pts = np.array([0,0,0])
        self.colors = np.array([0,0,0])
        mon_pcl2 = self.xyzrgb2pc(xyz=self.pts,bgr=self.colors,frame_id='base_link',time2use=self.telemeter_point.header.stamp)
        self.pub_pcl2.publish(mon_pcl2)            
        self.pcl2.header.stamp = self.telemeter_point.header.stamp
        self.pub_pcl2.publish(self.pcl2)
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
        if (self.pan_velocity_flag or self.tilt_velocity_flag):
            self.pan_velocity_flag = False
            self.tilt_velocity_flag = False
            try :
                if self.acq_seq : #ajout de points uniquement lors d'une acquisition
                    self.doses.append(self.ddd)
                    self.listener.waitForTransform("base_link", "collimator_link", self.collimator_point.header.stamp, rospy.Duration(1))
                    self.base_point=self.listener.transformPoint("base_link",self.collimator_point)
                    self.base_point.header.stamp=self.collimator_point.header.stamp
                    """
                    pts_new = np.array([float(self.base_point.point.x),float(self.base_point.point.y),float(self.base_point.point.z)])
                    self.pts = np.vstack((self.pts,pts_new))
                    r = int(255*self.ddd/self.cps_max)
                    g = int(255-255*self.ddd/self.cps_max)
                    b = 0
                    colors_new = np.array([b,g,r])
                    #print colors_new 
                    self.colors = np.vstack((self.colors,colors_new))
                    
                    mon_pcl2 = self.xyzrgb2pc(xyz=self.pts,bgr=self.colors,frame_id='base_link',time2use=self.telemeter_point.header.stamp)
                    
                    #mon_pcl2_dose = 
                    self.pub_pcl2.publish(mon_pcl2)
                    
                    # PCL
                    """
                    self.pcl.header.stamp = self.collimator_point.header.stamp
                    self.pcl.points.append(Point32(self.base_point.point.x,self.base_point.point.y,self.base_point.point.z))
                    self.process_cpl_flag = True
                    #channel_dose = ChannelFloat32('dose',Float32(self.ddd))
                    #if channel_dose is None :
                    #channel_dose = ChannelFloat32('dose', [Float32(self.ddd)])
                    #msg.channels.append(channel_dose)
                    #channel_dose.name = "dose"
                    #channel_dose.values = [Float32(self.ddd)]
                    #self.pcl.channels = (channel_dose)
                 
                    #self.pcl.channels = ChannelFloat32('dose', [Float32(self.doses)])
                    #self.pcl.channels= ChannelFloat32(name='dose', values=[Float32(self.doses)])
                    #print "self.pcl",self.pcl
                   
                    #self.pub_pcl_dose.publish(self.pcl)
                    #print "self.pcl",self.pcl
                    #self.process_pcl(force=True)
                """
                #ESSAI
                try:
                    self.pcl.header.stamp = rospy.Time.now()
                    self.listener_camera.waitForTransform('camera_link', 'base_link', rospy.Time.now(), rospy.Duration(1.0))
                    #base_point.header.stamp = rospy.Time.now()
                    print "PointCloud size",len(self.pcl.points)
                    print "Go transformPointCloud"
                    start_time = timeit.default_timer()
                    camera_pcl = self.listener_camera.transformPointCloud('camera_link',self.pcl)
                    elapsed = timeit.default_timer() - start_time
                    print "Time of transformPointCloud",elapsed
                except:
                    print "Camera PCL TF error"
                    raise
                #print "camera_pcl",camera_pcl
               
                #print camera_pcl
                msg = Dose_PCL()
                msg.dose_rate = self.doses
                self.pub_pcl_dose.publish(msg)
                self.pub_pcl_camera.publish(camera_pcl)
                """
                #camera_pcl.header.stamp = self.dose_rate_pcl.header.stamp
                #
                #self.pub_pcl2_dose.publish()
                #self.pcl2.header.stamp = self.telemeter_point.header.stamp
                #self.pub_pcl2.publish(self.pcl2)
                #print self.pcl2
            except:
                print "ERROR  acquisition"
                raise        
    
    def process_pcl(self,force=False):
        if (self.process_cpl_flag or force):
            self.process_cpl_flag = False
            try:
                self.pcl.header.stamp = rospy.Time.now()
                self.listener_camera.waitForTransform('camera_link', 'base_link', rospy.Time.now(), rospy.Duration(1.0))
                #base_point.header.stamp = rospy.Time.now()
                print "PointCloud size",len(self.pcl.points)
                print "Go transformPointCloud"
                start_time = timeit.default_timer()
                self.camera_pcl = self.listener_camera.transformPointCloud('camera_link',self.pcl)
                elapsed = timeit.default_timer() - start_time
                print "Time of transformPointCloud",elapsed
            except:
                print "Camera PCL TF error"
                raise
            #print "camera_pcl",camera_pcl
         
        #print camera_pcl
        msg = Dose_PCL()
        msg.dose_rate = self.doses
        self.pub_pcl_dose.publish(msg)
        self.pub_pcl_camera.publish(self.camera_pcl)
                
        
    def cb_telemeter(self,msg):
        if (msg.range>(self.telemeter_point.point.x+0.01)) | (msg.range<(self.telemeter_point.point.x-0.01)) :
            if msg.range!=0:            
                self.telemeter_point.header.stamp=msg.header.stamp
                self.telemeter_point.point.x=msg.range
                #self.pcl2_acqui()
                self.collimator_point.header.stamp = self.telemeter_point.header.stamp
                self.collimator_point.point.x = self.telemeter_point.point.x
                 
    def cb_GR1(self,msg):
        self.ddd=msg.count_rate
        self.pcl2_acqui()
        

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
               
    def __delete__(self):
        self.tele_inter.close()
        print "Connexion fermee"
        
   
if __name__=="__main__":

    print "Starting pcl2 creator node"
    rospy.init_node("pcl_creator_node")
    my_pcl2_creator=pcl2_creator()
    """
    rate=rospy.Rate(50)
    sleep(1)
    rospy.spin()
    """
    sleep(2)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        my_pcl2_creator.process_pcl()
        r.sleep()
    rospy.spin()
    rospy.loginfo("my_pcl_creator Node terminated")
