#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  7 22:03:58 2015
Test image grid
"""
__author__ = "Julien Favrichon, Jérémy Seyssaud"
__copyright__ = "Copyright 2015, CEA"
__license__ = ""
__version__ = "1.0.7"
__status__ = "Prototype"


import rospy
import cv2
import sys
import numpy as np
import math
from time import sleep
import rospkg
import os

from sensor_msgs.msg import Image,CompressedImage, CameraInfo, PointCloud2, Range, PointField, PointCloud
from cv_bridge import CvBridge, CvBridgeError

from dynamixel_msgs.msg import JointState

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, TransformStamped
from tourelle_ros.msg import Dose_rate, GR1_msg,Dose_PCL
from tourelle_ros.srv import SetBool,SetBoolRequest,SetBoolResponse,SetGrid, SetGridResponse, SetGridRequest,SetTransparency,SetTransparencyResponse,SetTransparencyRequest,SetScale,SetScaleRequest,SetScaleResponse

import tf

from image_geometry import PinholeCameraModel

#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import random
import timeit

from lib_tourelle.lib_geometry import *
  
class ImageAR:

    def __init__(self, grid_h=10, grid_v=10, collimator=10):
        self.image_pub = rospy.Publisher("/tourelle/image_ar",Image,queue_size=1)
        self.aof_h = 64.5 # horizontal angle of field (deg)
        self.aof_v = 38.3 # verical angle of field (deg)
        self.grid_h = grid_h
        self.grid_v = grid_v
        self.rows = 0
        self.cols = 0
        self.color = (0,255,0)
        self.transparency = 0.7
        self.grid_thickness = 1
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
                    
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
        
        self.camera_tele_point = PointStamped()
        self.camera_tele_point.header.stamp = rospy.Time.now()
        self.camera_tele_point.header.frame_id = 'camera_link'
        self.camera_tele_point.point.x = 0
        self.camera_tele_point.point.y = 0
        self.camera_tele_point.point.z = 0
        
        self.camera_ddd_point = PointStamped()
        self.camera_ddd_point.header.stamp = rospy.Time.now()
        self.camera_ddd_point.header.frame_id = 'camera_link'
        self.camera_ddd_point.point.x = 0
        self.camera_ddd_point.point.y = 0
        self.camera_ddd_point.point.z = 0
        
        self.base_point = PointStamped()
        self.base_point.header.stamp = rospy.Time.now()
        self.base_point.header.frame_id = 'base_link'
        self.base_point.point.x = 0
        self.base_point.point.y = 0
        self.base_point.point.z = 0
       
        self.base_point_cloud = []
        
        self.cam_model = PinholeCameraModel()
                
        self.telemeter_range = None
        
        self.tele_tf = True
        self.process_grid = True
        self.im_grid = None
        self.display_grid = True
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 1

        self.font_thickness = 2

        self.thickness = 2
        
        self.scale_font_scale = 0.5
        self.scale_font_thickness = 2
        
        self.cloud_label_font_scale = 0.5
        self.cloud_label_font_thickness = 1
        
        self.listener_tele = tf.TransformListener()
        self.listener_collimator = tf.TransformListener() 
        
        # TF for point cloud projections
        self.listener_camera = tf.TransformListener()
        self.listener_base = tf.TransformListener()
        
        self.camera_pcl = None
        
        self.get_cam_info = True
        
        self.dose_rate = GR1_msg()
        self.dose_rate_past = GR1_msg()
        self.dose_rate_past.dose_rate = 0
        self.dose_rate_past.count_rate = 0
        
        self.dose_rate_tab = []      
        
        # LUT for DED
        rp = rospkg.RosPack()
        color_file = os.path.join(rp.get_path('tourelle_ros'), 'scripts', 'colormap')
        #lut_file = np.loadtxt(r'colormap',dtype=np.uint8) #RGB
        lut_file = np.loadtxt(color_file,dtype=np.uint8) #RGB
        self.lut_bgr = np.fliplr(lut_file) #BGR
        self.lut_cv2 = self.lut_bgr.reshape(256,1,3)

        self.tourelle_pan_isMoving = False
        self.tourelle_tilt_isMoving = False
        
        self.collimator = collimator
        self.past_dose_rate = 0
        self.past_count_rate = 0      

        self.integrate_ddd = False
        self.delta_ddd_xyz = 2
        
        #self.DOSE_MAX = 100 #GR1_K * GR1_CONTRATE_MAX
        GR1_COUNTRATE_MAX = rospy.get_param("/tourelle/GR1_node/GR1_COUNTRATE_MAX",10.0)
        GR1_K = rospy.get_param("/tourelle/GR1_node/GR1_K",1.0) #etalonnage H*10 en microSv/h / cps
        self.DOSE_MAX = GR1_COUNTRATE_MAX * GR1_K
        self.DOSE_MIN = 0.0
        
        self.draw_pcl_data = False
        self.draw_pcltopic_data = True
        self.draw_pcl2topic_data = False
        
        self.debug_timeit = False
        self.display_cloud = True
        self.display_scale = True
        self.im_scale= None
        self.process_scale = True
        self.scale_height = 20
        self.scale_width = 255
        self.ellipse_correction = False
        
        self.m_MA = None
        self.m_alpha = None
        
        self.xc = None
        self.yc = None
        
        self.highter_behind = False
        self.display_cloud_label = False
        self.merge_doses = False
        
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
        
        #Creation des services
        print "Creation du service set_grid"
        self.service_setgrid = rospy.Service("/tourelle/image_ar/set_grid",SetGrid,self.cb_service_setgrid)
        print "Creation du service grid_enable"
        self.service_grid_enable = rospy.Service("/tourelle/image_ar/grid_enable",SetBool,self.cb_service_grid_enable)
        print "Creation du service set_transparency"
        self.service_set_transparency = rospy.Service("/tourelle/image_ar/set_transparency",SetTransparency,self.cb_service_set_transparency)
        print "Creation du service display_cloud"
        self.service_set_transparency = rospy.Service("/tourelle/image_ar/display_cloud",SetBool,self.cb_service_display_cloud)
        print "Creation du service set_scale"
        self.service_set_scale = rospy.Service("/tourelle/image_ar/set_scale",SetScale,self.cb_service_set_scale)
        print "Creation du service display_scale"
        self.service_display_scale = rospy.Service("/tourelle/image_ar/display_scale",SetBool,self.cb_service_display_scale)
        print "Creation du service ellipse_correction"
        self.service_ellipse_correction = rospy.Service("/tourelle/image_ar/ellipse_correction",SetBool,self.cb_service_ellipse_correction)
        print "Creation du service highter_behind"
        self.service_highter_behind = rospy.Service("/tourelle/image_ar/highter_behind",SetBool,self.cb_service_highter_behind)
        print "Creation du service display_cloud_label"
        self.service_display_cloud_label = rospy.Service("/tourelle/image_ar/display_cloud_label",SetBool,self.cb_service_display_cloud_label)
        print "Creation du service merge_doses"
        self.service_merge_doses = rospy.Service("/tourelle/image_ar/merge_doses",SetBool,self.cb_service_merge_doses)
        
        #self.sub_tilt = rospy.Subscriber("/tourelle/j_ACC_PLT_position_controller/state",JointState, self.cb_tilt)
        #self.sub_pan = rospy.Subscriber("/tourelle/j_BASE_ACC_position_controller/state",JointState, self.cb_pan)


        # Suscribers
        self.camera_info_sub = rospy.Subscriber("/tourelle/usb_cam_node/camera_info",CameraInfo,self.cb_camera_info)
        rospy.timer.sleep(1)
        self.image_sub = rospy.Subscriber("/tourelle/usb_cam_node/image_raw",Image,self.cb_im)
	#self.image_sub = rospy.Subscriber("/tourelle/usb_cam_node/image_raw/compressed",CompressedImage,self.cb_im)
              
        self.sub_telemeter = rospy.Subscriber("/tourelle/telemeter",Range, self.cb_telemeter)
        self.sub_dose_rate = rospy.Subscriber("/tourelle/GR1",GR1_msg, self.cb_dose_rate)  
        
        if self.draw_pcl2topic_data: self.sub_dose_rate_pcl2 = rospy.Subscriber("/tourelle/pcl2",PointCloud2, self.cb_dose_rate_pcl2)
        if self.draw_pcltopic_data :
            self.sub_dose_rate_pcl = rospy.Subscriber("/tourelle/pcl_dose",Dose_PCL, self.cb_dose_rate_pcl)
            self.sub_dose_rate_camera = rospy.Subscriber("/tourelle/pcl_camera",PointCloud, self.cb_camera_pcl)
            self.sub_acq = rospy.Subscriber("/tourelle/pcl_acq",Bool, self.cb_pcl_acq)
            
            self.dose_rate_pcl = Dose_PCL()
            self.camera_pcl = PointCloud()
            self.pcl_acq = True
    
    #@staticmethod
    def _getAlpha(self,y,x):
        C = Point(self.xc,self.yc)
        P1 = Point(x,y)
        v_CP1 = C.vecteur(P1)
        xmax = Point(self.cam_model.width,self.yc)
        v_Cxmax = C.vecteur(xmax)
        alpha = np.rad2deg(np.arccos((v_Cxmax*v_CP1)/(v_Cxmax.norme()*v_CP1.norme())))
        det = v_Cxmax.det(v_CP1)
        #signe = -1 if det <= 0 else 1
        signe = np.sign(det)
        return alpha * signe 

    def _getEllipseMatrices(self):
        size = self.cam_model.height, self.cam_model.width

        m_beta = np.fromfunction(lambda i, j: 0.5*self.aof_h/self.xc*np.sqrt((j-self.xc)**2+(i-self.yc)**2), size, dtype=np.float32)
        m_e = np.abs(np.sin(m_beta* np.pi / 180.)/np.cos(self.collimator* np.pi / 180.))
        self.m_MA = np.rint(2*self.radius/np.sqrt(1-np.power(m_e,2)))
        
        np.vectorize(self._getAlpha)
        self.m_alpha = np.fromfunction(self._getAlpha, size, dtype=np.float32)
         
    def _getRadius(self):
        self.radius = (self.collimator / 2.0) * self.cam_model.width / self.aof_h
        return self.radius
    
    def cb_service_merge_doses(self,msg):
        self.merge_doses = msg.data
        return SetBoolResponse()
        
    def cb_service_display_cloud_label(self,msg):
        self.display_cloud_label = msg.data
        return SetBoolResponse()
        
    def cb_service_ellipse_correction(self,msg):
        self.ellipse_correction = msg.data
        return SetBoolResponse()
        
    def cb_service_display_cloud(self,msg):
        self.display_cloud = msg.data        
        return SetBoolResponse()
        
    def cb_service_setgrid(self,msg):
        self.setGrid(msg.grid_h,msg.grid_v)
        return SetGridResponse()

    def cb_service_grid_enable(self,msg):
        self.display_grid = msg.data
        return SetBoolResponse()
        
    def cb_service_set_transparency(self,msg):
        self.transparency = float(msg.transparency / 100.0)
        return SetTransparencyResponse()
        
    def cb_service_set_scale(self,msg):
        # TODO
        self.DOSE_MAX = msg.scale_max * 1000 #(mSv/h -> uSv/h)
        self.DOSE_MIN = msg.scale_min * 1000 #(mSv/h -> uSv/h)
        self.process_scale = True
        #process sclae?
        return SetScaleResponse()
        
    def cb_service_display_scale(self,msg):
        self.display_scale = msg.data
        return SetBoolResponse()
        
    def cb_service_highter_behind(self,msg):
        self.highter_behind = msg.data
        return SetBoolResponse()
  
    def cb_pan(self,msg):
        self.tourelle_pan_isMoving = True if msg.is_moving() == True else False
        #print msg.is_moving()
        sleep(0.001)

    def cb_tilt(self,msg):
        self.tourelle_tilt_isMoving = True if msg.is_moving() == True else False
        sleep(0.001)
        
    def cb_pcl_acq(self,msg):
        self.pcl_acq = msg.data
        sleep(0.1)
                       
    def setGrid(self,grid_h,grid_v):
        self.grid_h = grid_h
        self.grid_v = grid_v
        self.process_grid = True
    
    def cb_camera_info(self,data):
        if self.get_cam_info:
            self.cam_model.fromCameraInfo(data)
            
            print "CAM INFO height= ",self.cam_model.height
            if self.cam_model.height != 0 :
                self.get_cam_info = False
                print "_getRadius"
                self._getRadius()
                print "radius =", self.radius
                self.xc = self.cam_model.width / 2
                self.yc = self.cam_model.height / 2
                print "_getEllipseMatrices"
                self._getEllipseMatrices()
                # Point central
                
        sleep(0.001)
        #print 'cam_model = ', np.array(self.cam_model.intrinsicMatrix())
        #print 'cam tf = ', self.cam_model.tfFrame()
    
    @staticmethod
    def imageBlend(img1,img2,p0=None):

        # Create a ROI
        rows,cols,channels = img2.shape
        rows1,cols1,channels1 = img1.shape      
        
        if p0 is None:
            roi = img1[0:rows, 0:cols]
        else:
            (x0,y0) = p0
            roi = img1[y0:y0+rows,x0:x0+cols]
        # Now create a mask of the grid and create its inverse mask also
        img2gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)
        
        # Now black-out the area of the grid in ROI
        img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
        
        # Take only region of the grid from grid image.
        img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
        
        # Put grid in ROI and modify the main image
        dst = cv2.add(img1_bg,img2_fg)

        if p0 is None:
            img1[0:rows, 0:cols] = dst
        else:
            (x0,y0) = p0
            img1[y0:y0+rows,x0:x0+cols] = dst   
      
        return img1
                
    def cb_telemeter(self,msg):
        if (msg.range > (self.telemeter_point.point.x + 0.01)) or (msg.range < (self.telemeter_point.point.x - 0.01)) :
            if msg.range != 0:            
                self.telemeter_point.header.stamp = msg.header.stamp
                self.telemeter_point.point.x = msg.range
                
                self.collimator_point.header.stamp = self.telemeter_point.header.stamp
                self.collimator_point.point.x = self.telemeter_point.point.x
                
                if self.tele_tf:
                    self.getTeleTF()
                    #self.tele_tf = False
                    #print "TeleTF x = %f, y = %f, z = %f"%(float(self.camera_tele_point.point.x),float(self.camera_tele_point.point.y),float(self.camera_tele_point.point.z))

        sleep(0.001)
                    
    def cb_dose_rate(self,msg):
        
        #print "cb_dose_rate"
        try:
            self.getCollimatorTF()
            #print "CollimatorTF x = %f, y = %f, z = %f"%(float(self.camera_ddd_point.point.x),float(self.camera_ddd_point.point.y),float(self.camera_ddd_point.point.z))

        except:
            print "Can't process CollimatorTF"
        else:
            if (self.integrate_ddd): #integration des valeurs
                print "integration des valeurs"
                #distance = math.sqrt((self.camera_ddd_point_last.point.x - self.camera_ddd_point.point.x)**2 +(self.camera_ddd_point_last.point.y - self.camera_ddd_point.point.y)**2 + (self.camera_ddd_point_last.point.z-self.camera_ddd_point.point.z)**2)
                #print "distance ddd xyz",distance
                #if (distance < self.delta_ddd_xyz):
                if(not self.tourelle_tilt_isMoving and not self.tourelle_pan_isMoving):
                    self.dose_rate.dose_rate = (msg.dose_rate + self.dose_rate_past.dose_rate) / (msg.dose_rate.header.stamp.to_sec() - self.dose_rate_past.header.stamp.to_sec())
                    self.dose_rate.count_rate = (msg.count_rate + self.dose_rate_past.count_rate) / (msg.dose_rate.header.stamp.to_sec() - self.dose_rate_past.header.stamp.to_sec())
                else:
                    self.dose_rate.dose_rate = msg.dose_rate
                    self.dose_rate.count_rate = msg.count_rate
                self.dose_rate_past = self.dose_rate
            else:
                self.dose_rate.dose_rate = msg.dose_rate
                self.dose_rate.count_rate = msg.count_rate
        
        if self.draw_pcl_data == True:    
            try:
                self.getCollimatorTF_PtCloud()
                self.dose_rate_tab.append(self.dose_rate.dose_rate)
                print "Camera TF pt cloud len= ", len(self.base_point_cloud)
            except:
                print "Can't process collimatorTF_PtCloud"
        
        sleep(0.001)
#        print "dose_rate",self.dose_rate.dose_rate
#        print "count_rate",self.dose_rate.count_rate
        
    def cb_dose_rate_pcl(self,msg):
        if self.draw_pcltopic_data:
            self.dose_rate_pcl = msg
        sleep(0.001)
        
    def cb_camera_pcl(self,msg):
         if self.draw_pcltopic_data:
            self.camera_pcl = msg
         sleep(0.001)
        
    def cb_dose_rate_pcl2(self,msg):
        if self.draw_pcl2topic_data:
            self.pcl2 = msg
            self.getPCL2_TF()
        sleep(0.001)   
     
        
    def getTeleTF(self):
        try :
            self.listener_tele.waitForTransform(self.cam_model.tfFrame(), "laser_link", self.telemeter_point.header.stamp, rospy.Duration(1.0/20))
                
            self.camera_tele_point = self.listener_tele.transformPoint(self.cam_model.tfFrame(),self.telemeter_point)
             
            self.camera_tele_point.header.stamp = self.telemeter_point.header.stamp
            #pts_new = np.array([float(self.base_point.point.x),float(self.base_point.point.y),float(self.base_point.point.z)])
            """
            self.listener_tele.waitForTransform(self.cam_model.tfFrame(), "laser_link", self.cam_model.stamp, 1.0 / 30)
            self.camera_tele_point = self.listener_tele.transformPoint(self.cam_model.tfFrame(),self.telemeter_point)
            """
            #tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,acquisition_time, timeout);
            #tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,acquisition_time, transform);
            
        except:
            print "Tele TF error"
            #raise
          
    def getCollimatorTF(self):
        """
        Get the TF of in colimator coordinate system
        Hypothesis: the distance zone for collimated gamma sensor acquisition is approximatively the same than telementer
        """
        try :
            self.listener_collimator.waitForTransform(self.cam_model.tfFrame(), "collimator_link", self.collimator_point.header.stamp, rospy.Duration(1.0/20))
            #self.camera_ddd_point_last = self.camera_ddd_point
            self.camera_ddd_point = self.listener_collimator.transformPoint(self.cam_model.tfFrame(),self.collimator_point)
             
            self.camera_ddd_point.header.stamp = self.collimator_point.header.stamp
            #pts_new = np.array([float(self.base_point.point.x),float(self.base_point.point.y),float(self.base_point.point.z)])
          
        except:
            print "Collimator TF error"
            #raise
            
    def getCollimatorTF_PtCloud(self):
        """
        Get the TF of in colimator coordinate system
        Hypothesis: the distance zone for collimated gamma sensor acquisition is approximatively the same than telementer
        """
        try :
            self.listener_base.waitForTransform("base_link", "collimator_link", self.collimator_point.header.stamp, rospy.Duration(1.0/20))
            self.base_point = self.listener_base.transformPoint("base_link",self.collimator_point)
            self.listener_base.fromTranslationRotation()
            self.base_point.header.stamp = self.collimator_point.header.stamp
            self.base_point_cloud.append(self.base_point)
            print "len(self.base_point_cloud)",len(self.base_point_cloud)
            #pts_new = np.array([float(self.base_point.point.x),float(self.base_point.point.y),float(self.base_point.point.z)])
          
        except:
            print "Collimator TF error"
            #raise
            
    def getPCL2_TF(self):
        """
        Get the PCL2 TF in camera coordinate system
        """
        try :
            """
            self.listener_camera.waitForTransform("camera_link", "laser_link", self.collimator_point.header.stamp, rospy.Duration(1.0/20))
            #self.camera_pcl = self.listener_camera.transformPoint("camera_link",self.collimator_point)
            pcl =self.pcl2           
            self.camera_pcl = self.listener_camera.transformPointCloud("camera_link",pcl)
            """
            cloud_in =self.pcl2
            transform = TransformStamped()
            transform.header.frame_id = "camera_link"
            transform.header.stamp = rospy.Time.now()
            cloud_out = do_transform_cloud(PointCloud2(cloud_in), transform)
           
            print "Camera PCL2 TF Ok"
        except:
            print "Camera PCL2 TF error"
            raise
     
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
    
    def addTeleInfo(self,img):
        
        #tf::Point pt = transform.getOrigin();
        #cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
        #cv::Point2d uv;
        try:
            tele_center = self.cam_model.project3dToPixel((float(self.camera_tele_point.point.x),float(self.camera_tele_point.point.y),float(self.camera_tele_point.point.z)))

            #print "tele_center= ",tele_center
        except:
            print "Can't project telemeter info to 2D"
        else:
            #print "float(tele_center[0])",float(tele_center[0])
            if not math.isnan(float(tele_center[0])):            
                #print "int((tele_center[0])",int((tele_center[0]))
                #print "int((tele_center[1])",int((tele_center[1]))
                cv2.circle(img, (int(np.rint(tele_center[0])),int((tele_center[1]))),3, [0,0,255],-1)
                #cv2.circle(img, (200,200),3, [0,255,0],-1)
                
                #Label tele
                label = "tele:" + "{0:.2f}".format(self.telemeter_point.point.x) + " m"
                size = cv2.getTextSize(str(label), self.font, self.font_scale, self.font_thickness)[0]
                label_bottom_left = (int((tele_center[0] - size[0]-self.radius) ),int( (tele_center[1] + size[1]) ))
                cv2.putText(img, str(label), label_bottom_left, self.font, self.font_scale, (0, 0, 255), self.font_thickness)
                
                #Label tele
                """
                #surface de la zone ddd en fonction de la distance S = PI * (d*tan(angle_colli/2))**2
                label_zone_ddd = "area ddd:" + "{0:.4f}".format(math.pi * (self.telemeter_point.point.x * math.tan(math.radians(self.collimator/2)))**2) + " m2"
                size_zone_ddd = cv2.getTextSize(str(label_zone_ddd), self.font, self.font_scale, self.font_thickness)[0]                             
                label_top_left_screen = (10,2 * size_zone_ddd[1]+15)   
                cv2.putText(img, str(label_zone_ddd), label_top_left_screen, self.font, self.font_scale, (0, 0, 255), self.font_thickness)
                """
                #diametre de la zone ddd en fonction de la distance D = 2 * d*tan(angle_colli/2) * 100 (cm)
                label_zone_ddd = "diam ddd:" + "{0:.1f}".format(100.0 * 2.0 * self.telemeter_point.point.x * math.tan(math.radians(self.collimator/2.0))) + " cm"
                size_zone_ddd = cv2.getTextSize(str(label_zone_ddd), self.font, self.font_scale, self.font_thickness)[0]                             
                label_top_left_screen = (10,2 * size_zone_ddd[1]+15)   
                cv2.putText(img, str(label_zone_ddd), label_top_left_screen, self.font, self.font_scale, (0, 0, 255), self.font_thickness)
            
            #else:
                #print "tele_center is nan"
        finally:
            return img
    
    def _drawDDDTarget(self,img,center):
        if not math.isnan(float(center[0])):  
            #print "int((ddd_center[0])",int((center[0]))
            #print "int((ddd_center[1])",int((center[1]))
            cv2.circle(img, (int(np.rint(center[0])),int(np.rint(center[1]))),3, [255,0,0],-1)
            cv2.circle(img, (int(np.rint(center[0])),int(np.rint(center[1]))),int(self.radius + 1), [255,0,0],2)
        return img
    
    def _drawScale(self,img):
        size = 20, 255, 1 # taille de la barre d'échelle hauteur*Largeur
        im_scale = np.zeros(size, dtype=np.uint8)
        #im_scale_mask = np.zeros(size, dtype=np.uint8)      
        #im_scale_mask[:,:] = 255
        for i in range(254,0,-1):
            im_scale[:,i] = i
        #print im_scale
        im_scale_color = cv2.cvtColor(im_scale, cv2.COLOR_GRAY2BGR)
        im_scale_color_lut = cv2.LUT(im_scale_color,self.lut_cv2)
        #img = cv2.add(img,im_scale_color_lut,mask = im_scale_mask)
        x0 = 50        
        y0 = self.cam_model.height - 20 - 5
        img = self.imageBlend(img,im_scale_color_lut,(x0,y0))
        return img
        
    def addScale(self,img):
        x0 = 20        
        y0 = self.cam_model.height - 20 - 5
              
        #label_min = "0 mSv/h"
        label_min = "{0:.3f}".format(self.DOSE_MIN/1000.0) + " mSv/h"
        label_max = "{0:.3f}".format(self.DOSE_MAX/1000.0) + " mSv/h"
        size_zone_min = cv2.getTextSize(str(label_min), self.font, self.scale_font_scale, self.scale_font_thickness)[0] 
        size_zone_max = cv2.getTextSize(str(label_max), self.font, self.scale_font_scale, self.scale_font_thickness)[0] 
        label_min_pos = (20,self.cam_model.height-20-size_zone_min[1]-5)  
        label_max_pos = (20 + 255,self.cam_model.height - 20 - size_zone_max[1] - 5)
        cv2.putText(img, str(label_min), label_min_pos, self.font, self.scale_font_scale, (0, 0, 255), self.scale_font_thickness)
        cv2.putText(img, str(label_max), label_max_pos, self.font, self.scale_font_scale, (0, 0, 255), self.scale_font_thickness)
        
        img = self.imageBlend(img,self.im_scale,(x0,y0))
        return img
    
    def buildScaleLayer(self):
        size = 20, 255, 1 # taille de la barre d'échelle hauteur*Largeur
        size = self.scale_height,self.scale_width,1
        im_scale_gray = np.zeros(size, dtype=np.uint8)
        #im_scale_mask = np.zeros(size, dtype=np.uint8)      
        #im_scale_mask[:,:] = 255
        for i in range(254,0,-1):
            im_scale_gray[:,i] = i
        #print im_scale
        im_scale_color = cv2.cvtColor(im_scale_gray, cv2.COLOR_GRAY2BGR)
        self.im_scale = cv2.LUT(im_scale_color,self.lut_cv2)
            
    def _drawDDDActivity(self,img,center,dose_rate,trans):
        size = self.rows, self.cols, 1
        im_ddd = np.zeros(size, dtype=np.uint8)
        im_ddd[:,:] = 0
        im_ddd_mask = np.zeros(size, dtype=np.uint8)
        im_ddd_mask[:,:] = 0
      
        #color = dose_rate * 255.0 / self.DOSE_MAX
        #color = dose_rate * 255.0 / (self.DOSE_MAX - self.DOSE_MIN)
        color = (dose_rate - self.DOSE_MIN) * 255.0 / (self.DOSE_MAX - self.DOSE_MIN)
        if color < 0 : color = 0
        if color > 255 : color = 255
        
        #print "color= ",  color
        cv2.circle(im_ddd, (int(np.rint(center[0])),int(np.rint(center[1]))),int(self.radius), int(color),-1)
        cv2.circle(im_ddd_mask, (int(np.rint(center[0])),int(np.rint(center[1]))),int(self.radius), 255,-1)
        # Couleur Fonction de la dose + cercle plein transparent
        # LUT
        im_ddd_color = cv2.cvtColor(im_ddd, cv2.COLOR_GRAY2BGR)
        im_ddd_color_lut = cv2.LUT(im_ddd_color,self.lut_cv2)
                    
        img2 = cv2.addWeighted(img,1-trans,im_ddd_color_lut,trans,0)
               
        img_ddd_pre = cv2.add(img,img2,mask = im_ddd_mask)
                   
        img = self.imageBlend(img,img_ddd_pre)
        return img
    
    def _drawDDDActivityArray(self,img,centers,dose_rates,trans,ellipse_correction=False,labels=False,merge_doses=False):
        size = self.rows, self.cols, 1
        im_ddd = np.zeros(size, dtype=np.uint8)
        im_ddd[:,:] = 0
        im_ddd_mask = np.zeros(size, dtype=np.uint8)
        im_ddd_mask[:,:] = 0
        
        if merge_doses :
            im_ddd_dose= np.zeros(size, dtype=np.float32)
            im_ddd_merge = np.zeros(size, dtype=np.float32)
            #im_ddd_dose_count = np.ones(size, dtype=np.uint8)
            im_ddd_dose_count = np.zeros(size, dtype=np.uint8)
            im_ddd_merge_count = np.zeros(size, dtype=np.uint8)
        
        ma_liste = zip(centers,dose_rates)
        #On passe les points le plus "chauds" en dernier si self.highter_behind est False -> Tri par rapport à la 2ieme colonne du tuple
        ddd_points_sorted = sorted(ma_liste, key=lambda colonnes: colonnes[1], reverse=self.highter_behind)   
               
        #for center,dose_rate in zip(centers,dose_rates):
        for center,dose_rate in ddd_points_sorted:
        
            if not math.isnan(float(center[0])): 
                #color = dose_rate * 255.0 / self.DOSE_MAX
                #color = dose_rate * 255.0 / (self.DOSE_MAX - self.DOSE_MIN)
                color = (dose_rate - self.DOSE_MIN) * 255.0 / (self.DOSE_MAX - self.DOSE_MIN)
                if color < 0 : color = 0
                #print "color= ",  color
                x = int(np.rint(center[0]))
                y = int(np.rint(center[1]))
                # Test que le centre soit dans l'écran
                # TODO elargir les matrices ellipse pour les centre en bordure exterieur puissent êtes utilisé
                if x>=0 and x <= self.cam_model.width-1 and y >=0 and y <= self.cam_model.height-1:
                    
                    if ellipse_correction and not (x == self.xc and y == self.yc): #ellispe correction 
                        #print x,y
                        #test = 'ok' if self.m_alpha is None else None
                        #print "self.m_alpha",test
                        ellipse = (x,y),(self.m_MA[y,x],2*self.radius),self.m_alpha[y,x]
                        if merge_doses :
                            cv2.ellipse(im_ddd_dose,ellipse,dose_rate,-1)
                            cv2.ellipse(im_ddd_merge_count,ellipse,1,-1)
                            im_ddd_merge += im_ddd_dose
                            im_ddd_dose_count += im_ddd_merge_count
                            im_ddd_dose[:,:] = 0.0
                            im_ddd_merge_count[:,:] = 0.0
                        else:
                            cv2.ellipse(im_ddd,ellipse,int(color),-1)
                        cv2.ellipse(im_ddd_mask,ellipse,255,-1)
                    else:
                        if merge_doses :
                            cv2.circle(im_ddd_dose,(x,y),int(self.radius),dose_rate,-1)
                            cv2.circle(im_ddd_merge_count,(x,y),int(self.radius),1,-1)
                            im_ddd_merge += im_ddd_dose
                            im_ddd_dose_count += im_ddd_merge_count
                            im_ddd_dose[:,:] = 0.0
                            im_ddd_merge_count[:,:] = 0.0
                        else:
                            cv2.circle(im_ddd,(x,y),int(self.radius), int(color),-1)
                        cv2.circle(im_ddd_mask,(x,y),int(self.radius), 255,-1)
               
            else:
                print "ddd_center of point cloud is nan"
        if merge_doses : 
            old_set=np.seterr(divide='ignore',invalid='ignore')
            im_ddd_merge /= im_ddd_dose_count
            #im_ddd_merge *= 255.0 / self.DOSE_MAX
            #im_ddd_merge *= 255.0 / (self.DOSE_MAX - self.DOSE_MIN)
            im_ddd_merge = (im_ddd_merge - self.DOSE_MIN) * 255.0 / (self.DOSE_MAX - self.DOSE_MIN)
            im_ddd_merge[im_ddd_merge < 0] = 0
            im_ddd = im_ddd_merge.astype(np.uint8)
            
            np.seterr(**old_set)
            #im_ddd_merge /= im_ddd_dose_count * 255.0 / self.DOSE_MAX
            #im_ddd = im_ddd_merge.astype(np.uint8)
            
        # Couleur Fonction de la dose + cercle plein transparent
        # LUT
        im_ddd_color = cv2.cvtColor(im_ddd, cv2.COLOR_GRAY2BGR)
        im_ddd_color_lut = cv2.LUT(im_ddd_color,self.lut_cv2)
                    
        img2 = cv2.addWeighted(img,1-trans,im_ddd_color_lut,trans,0)
       
        img_ddd_pre = cv2.add(img,img2,mask = im_ddd_mask)
                           
        img = self.imageBlend(img,img_ddd_pre)
        
        if labels:
            for center,dose_rate in ddd_points_sorted:
                x = int(np.rint(center[0]))
                y = int(np.rint(center[1]))
                label = "{0:.3f}".format(dose_rate/1000.0) + " mSv" #dose_rate = H*10
                #size = cv2.getTextSize(str(label), self.font, self.cloud_label_font_scale, self.cloud_label_font_thickness)[0]
                label_pos = (x,y)
                #label_top_left = (150,150)
                cv2.putText(img, str(label), label_pos, self.font, self.cloud_label_font_scale, (255, 0, 0), self.cloud_label_font_thickness)
        return img    
    """    
    def _drawDDDActivityArray_back(self,img,ddd_points):
        size = self.rows, self.cols, 1
        im_ddd = np.zeros(size, dtype=np.uint8)
        im_ddd[:,:] = 0
        im_ddd_mask = np.zeros(size, dtype=np.uint8)
        im_ddd_mask[:,:] = 0
        
        #On passe les points le plus "chauds" en dernier -> Tri par rapport à la 3ieme colonne du tuple
        lambda colonnes: colonnes[2]
        ddd_points_sorted = sorted(ddd_points, key=lambda colonnes: colonnes[2])
        #print "ddd_points_sorted",ddd_points_sorted
        #print "ddd_points",ddd_points
        for ddd_point in ddd_points_sorted:
            
            if not math.isnan(float(ddd_point[0])): 
                color = ddd_point[2] * 255.0 / self.DOSE_MAX
                #print "color= ",  color
                cv2.circle(im_ddd, (int(np.rint(ddd_point[0])),int(np.rint(ddd_point[1]))),int(self.radius), int(color),-1)
                cv2.circle(im_ddd_mask, (int(np.rint(ddd_point[0])),int(np.rint(ddd_point[1]))),int(self.radius), 255,-1)
            
            else:
                print "ddd_center of point cloud is nan"
        # Couleur Fonction de la dose + cercle plein transparent
        # LUT
        im_ddd_color = cv2.cvtColor(im_ddd, cv2.COLOR_GRAY2BGR)
        im_ddd_color_lut = cv2.LUT(im_ddd_color,self.lut_cv2)
                    
        img2 = cv2.addWeighted(img,0.3,im_ddd_color_lut,0.7,0)
       
        img_ddd_pre = cv2.add(img,img2,mask = im_ddd_mask)
                   
        img = self.imageBlend(img,img_ddd_pre)
        return img  
    """    
    def addDDDInfo(self,img):
        
        #tf::Point pt = transform.getOrigin();
        #cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
        #cv::Point2d uv;
        try:
            ddd_center = self.cam_model.project3dToPixel((float(self.camera_ddd_point.point.x),float(self.camera_ddd_point.point.y),float(self.camera_ddd_point.point.z)))
           
        except:
            print "Can't project collimator info to 2D"
        else:
            #print "ddd_center = ",ddd_center
            if not math.isnan(float(ddd_center[0])):  
                img = self._drawDDDTarget(img,ddd_center)
                # Couleur Fonction de la dose + cercle plein transparent
                # LUT
                """
                color = [255,255,0] #bgr
                size = self.rows, self.cols, 3
                im_ddd = np.zeros(size, dtype=np.uint8)
                im_ddd[:,:] = (0,0,0)
                """
                dose_rate = self.dose_rate.dose_rate
                count_rate = self.dose_rate.count_rate
                
                """
                if (self.integrate_ddd): #integration des valeurs
                    #faux if ((math.sqrt((self.past_ddd_center[0]-ddd_center[0])**2 + (self.past_ddd_center[1]-ddd_center[1])**2)) < self.delta_ddd_uv):
                    pass 
                else:
                    dose_rate = self.dose_rate.dose_rate
                    count_rate = self.dose_rate.count_rate
                """
  
                img = self._drawDDDActivity(img,ddd_center,dose_rate,self.transparency)
       
                label = "dose_rate:" + "{0:.3f}".format(dose_rate/1000) + " mSv" #dose_rate = H*10
                #label += "count_rate:" + "{0:.2f}".format(count_rate) + " cp/s" #count_rate)nb coup/s
                #print "label",label
                size = cv2.getTextSize(str(label), self.font, self.font_scale, self.font_thickness)[0]
                #print "size",size
                #label_top_left = (int((ddd_center[0] - size[0] + self.radius)),int((ddd_center[1] - size[1]+ self.radius)))
                label_top_left_screen = (10,size[1]+5)
                #label_top_left = (150,150)
                cv2.putText(img, str(label), label_top_left_screen, self.font, self.font_scale, (255, 0, 0), self.font_thickness)
                
            #else:
                #print "ddd_center is nan"
        finally:
            return img    
            
    def addDDDInfoCloud(self,img):
        
        #tf::Point pt = transform.getOrigin();
        #cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
        #cv::Point2d uv;
        if self.draw_pcl_data:
            camera_point_cloud = []
            for base_point in self.base_point_cloud:
                print "base_point",base_point
                try :
                    self.listener_camera.waitForTransform(self.cam_model.tfFrame(), "base_link", rospy.Time(0), rospy.Duration(1.0/20))
                    #base_point.header.stamp = rospy.Time.now()
                    camera_point = self.listener_camera.transformPoint(self.cam_model.tfFrame(),base_point)
                    camera_point.header.stamp = base_point.header.stamp
                    camera_point_cloud.append(camera_point)
                except:
                    print "Camera cloud TF error"
                    camera_point_cloud.append(None)
                    raise
        
            for camera_point,dose_rate in camera_point_cloud,self.dose_rate_tab:
                if camera_point is not None:
                    try:
                        ddd_center = self.cam_model.project3dToPixel((float(camera_point.point.x),float(camera_point.point.y),float(camera_point.point.z)))
                            
                    except:
                        print "Can't project point cloud  to 2D"
                    else:
                        #print "ddd_center = ",ddd_center
                        if not math.isnan(float(ddd_center[0])):  
                            cv2.circle(img, (int(np.rint(ddd_center[0])),int(np.rint(ddd_center[1]))),3, [255,0,0],-1)
                            #self._drawDDDActivity(img,ddd_center,dose_rate)
                        else:
                            print "ddd_center of point cloud is nan"
                            
        if self.draw_pcltopic_data:
            """
            try :
                self.listener_camera.waitForTransform(self.cam_model.tfFrame(), "base_link", self.dose_rate_pcl.header.stamp, rospy.Duration(1.0/20))
                #base_point.header.stamp = rospy.Time.now()
                print "Go transformPointCloud"                
                camera_pcl = self.listener_camera.transformPointCloud(self.cam_model.tfFrame(),PointCloud(self.dose_rate_pcl))
                camera_pcl.header.stamp = self.dose_rate_pcl.header.stamp
            except:
                print "Camera PCL TF error"
                raise
            else:
                print "Camera PCL TF OK"
                for camera_point in camera_pcl.points:
                    if camera_point is not None:
                        try:
                            ddd_center = self.cam_model.project3dToPixel((float(camera_point.point.x),float(camera_point.point.y),float(camera_point.point.z)))
                                
                        except:
                            print "Can't project point cloud  to 2D"
                        else:
                            #print "ddd_center = ",ddd_center
                            if not math.isnan(float(ddd_center[0])):  
                                cv2.circle(img, (int(np.rint(ddd_center[0])),int(np.rint(ddd_center[1]))),3, [255,0,0],-1)
                                #self._drawDDDActivity(img,ddd_center,dose_rate)
                            else:
                                print "ddd_center of point cloud is nan"
            #print "camera_pcl",camera_pcl
            """
            camera_pcl = self.camera_pcl
            #print "camera_pcl",camera_pcl
            """#1ere methode
            for camera_point in camera_pcl.points:
                #print "camera_point",camera_point
                if camera_point is not None:
                    try:
                                              
                        ddd_center = self.cam_model.project3dToPixel((float(camera_point.x),float(camera_point.y),float(camera_point.z)))
                            
                    except:
                        print "Can't project point cloud  to 2D"
                        raise
                    else:
                        #print "ddd_center = ",ddd_center
                        if not math.isnan(float(ddd_center[0])):  
                            cv2.circle(img, (int(np.rint(ddd_center[0])),int(np.rint(ddd_center[1]))),3, [255,0,0],-1)
                            self._drawDDDActivity(img, (int(np.rint(ddd_center[0])),int(np.rint(ddd_center[1]))),10.)
                            #self._drawDDDActivity(img,ddd_center,dose_rate)
                        else:
                            print "ddd_center of point cloud is nan"
            """   
            
            #2nd methode
            ddd_centers = []
            dose_rates = []
            for camera_point in camera_pcl.points:
                #print "camera_point",camera_point
                if camera_point is not None:
                     # teste si la coposante en z du point est du meme signe que le point courant de la mesure pour savoir si l'on se trouve dans le meme demis sphère (pb de symétrie)
                         
                    if ((self.camera_ddd_point.point.z < 0 and camera_point.z < 0) or (self.camera_ddd_point.point.z > 0 and camera_point.z > 0 )):
                        try:
                            ddd_center = self.cam_model.project3dToPixel((float(camera_point.x),float(camera_point.y),float(camera_point.z)))
                                
                        except:
                            print "Can't project point cloud  to 2D"
                            raise
                            ddd_centers.append(None,None)
                        else:
                            ddd_centers.append((int(np.rint(ddd_center[0])),int(np.rint(ddd_center[1]))))
                            #dose_rates.append(random.randint(0, 100))
                            dose_rates = self.dose_rate_pcl.dose_rate
                        
            #print "ddd_center = ",ddd_center           
            self._drawDDDActivityArray(img,ddd_centers,dose_rates,self.transparency,self.ellipse_correction,self.display_cloud_label,self.merge_doses)
            """
            ddd_acq = [] #contiendra (x,y,dose)
            
            for camera_point in camera_pcl.points:
                #print "camera_point",camera_point
                if camera_point is not None:
                    try:
                                              
                        ddd_center = self.cam_model.project3dToPixel((float(camera_point.x),float(camera_point.y),float(camera_point.z)))
                            
                    except:
                        print "Can't project point cloud  to 2D"
                        raise
                        ddd_acq.append(None,None,None)
                    else:
                        print "self.dose_rate_pcl.dose_rate",self.dose_rate_pcl.dose_rate
                        ddd_acq.append((int(np.rint(ddd_center[0])),int(np.rint(ddd_center[1])),self.dose_rate_pcl.dose_rate))
                        
            #print "ddd_center = ",ddd_center
            self._drawDDDActivityArray_back(img,ddd_acq)
            """
        return img    
    
    def addSensorsInfo(self,img):
        img = self.addTeleInfo(img)
        #img = self.addDDDInfo(img)
    
    def buildGridLayer(self):
        
        cols = self.cols
        rows = self.rows
        """
        if self.im_grid is None:
            size = rows, cols, 3
            self.im_grid = np.zeros(size, dtype=np.uint8)
            self.im_grid[:,:] = (0,0,0)
        """
        size = rows, cols, 3
        self.im_grid = np.zeros(size, dtype=np.uint8)
        self.im_grid[:,:] = (0,0,0)
            
        scale_deg_px = float(cols /self.aof_h)
        #print "scale_deg_px",scale_deg_px
        center_x = self.cols / 2
        center_y = self.rows / 2
        
        #print "center_x",center_x
        #print "center_y",center_y
        
        cv2.circle(self.im_grid, (int(np.rint(center_x)),int(np.rint(center_y))),3, [0,255,0],-1)
        
        demi_delta_x = int(np.rint(0.5 * scale_deg_px * self.grid_v))
        
        indice_right = center_x + demi_delta_x    
        indice_left = center_x - demi_delta_x
        
        # 1ere lignes autour du centre
        if indice_right < cols :
            pt1 = (indice_right,0)
            pt2 = (indice_right,rows-1)
            cv2.line(self.im_grid,pt1,pt2,self.color,self.grid_thickness)
        
        if indice_left >= 0 :
            pt1 = (indice_left,0)
            pt2 = (indice_left,rows-1)
            cv2.line(self.im_grid,pt1,pt2,self.color,self.grid_thickness)
        
        #print "int(0.5 * self.aof_h/self.grid_h)",int(0.5*self.aof_h/self.grid_h)
        for x in range(1, int(0.5*self.aof_h/self.grid_h)+1):
            #col
            #print "x",x
            indice_right += 2 * demi_delta_x
            indice_left -= 2 * demi_delta_x
            #print "indice_right",indice_right
            #print "indice_left",indice_left
            if indice_right < cols :
                pt1 = (indice_right,0)
                pt2 = (indice_right,rows-1)
                cv2.line(self.im_grid,pt1,pt2,self.color,self.grid_thickness)
                
            if indice_left >= 0 :
                pt1 = (indice_left,0)
                pt2 = (indice_left,rows-1)
                cv2.line(self.im_grid,pt1,pt2,self.color,self.grid_thickness)
        
        # 1ere lignes autour du centre 
        demi_delta_y = int(np.rint(0.5 * scale_deg_px * self.grid_h))
        
        indice_hight = center_y + demi_delta_y  
        indice_low = center_y - demi_delta_y
        
        if indice_low >= 0 :
            pt1 = (0,indice_low)
            pt2 = (cols-1,indice_low)
            cv2.line(self.im_grid,pt1,pt2,self.color,self.grid_thickness)
                
        if indice_hight < rows :
            pt1 = (0,indice_hight)
            pt2 = (cols-1,indice_hight)
            cv2.line(self.im_grid,pt1,pt2,self.color,self.grid_thickness)
        
        #print "int(0.5 * self.aof_v/self.grid_v)",int(0.5*self.aof_v/self.grid_v)
        for y in range(1, int(0.5*self.aof_v/self.grid_v)+1 ):
            #row
            #print "y",y
            indice_hight += 2 * demi_delta_y
            indice_low -= 2 * demi_delta_y
            #print "indice_hight",indice_hight
            #print "indice_low",indice_low
            if indice_low >= 0 :
                pt1 = (0,indice_low)
                pt2 = (cols-1,indice_low)
                cv2.line(self.im_grid,pt1,pt2,self.color,self.grid_thickness)
                
            if indice_hight < rows :
                pt1 = (0,indice_hight)
                pt2 = (cols-1,indice_hight)
                cv2.line(self.im_grid,pt1,pt2,self.color,self.grid_thickness)

    def cb_im(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
          print e
        
        (self.rows,self.cols,channels) = cv_image.shape
     
        if self.process_grid and self.display_grid:
            self.buildGridLayer()
            print "Grid processed"
            self.process_grid = False
     
        if self.process_scale and self.display_scale:
            self.buildScaleLayer()
            print "Scale processed"
            self.process_scale = False
            
        #cv_image = self.addSensorsInfo(cv_image)
        if self.debug_timeit:
            elapsed1 = 0
            if self.display_cloud:
                start_time = timeit.default_timer()
                cv_image = self.addDDDInfoCloud(cv_image)
                elapsed1 = timeit.default_timer() - start_time
                print "Time of addDDDInfoCloud",elapsed1
            start_time = timeit.default_timer()
            cv_image = self.addTeleInfo(cv_image)
            elapsed2 = timeit.default_timer() - start_time
            print "Time of addTeleInfo",elapsed2
            start_time = timeit.default_timer()
            cv_image = self.addDDDInfo(cv_image)
            elapsed3 = timeit.default_timer() - start_time
            print "Time of addDDDInfo",elapsed3
            print "Total time:",elapsed1+elapsed2+elapsed3
        else:
            if self.display_cloud : cv_image = self.addDDDInfoCloud(cv_image)
            cv_image = self.addTeleInfo(cv_image)
            cv_image = self.addDDDInfo(cv_image)
            
        if self.display_scale: cv_image = self.addScale(cv_image)
               
        if self.display_grid: cv_image = self.imageBlend(cv_image,self.im_grid)

        if cv_image is not None:      
            try:
              self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError, e:
              print e
      
def main(args):
    rospy.init_node('Image_AR', anonymous=True)
    iar = ImageAR(grid_h=5, grid_v=5)
    
    # Lancement des services
    rospy.wait_for_service("/tourelle/image_ar/set_grid") 
    rospy.loginfo("Lancement du service set_grid")
    rospy.wait_for_service("/tourelle/image_ar/grid_enable") 
    rospy.loginfo("Lancement du service grid_enable")
    rospy.wait_for_service("/tourelle/image_ar/set_transparency") 
    rospy.loginfo("Lancement du service set_transparency")
    rospy.wait_for_service("/tourelle/image_ar/display_cloud") 
    rospy.loginfo("Lancement du service display_cloud")
    rospy.wait_for_service("/tourelle/image_ar/set_scale") 
    rospy.loginfo("Lancement du service set_scale")
    rospy.wait_for_service("/tourelle/image_ar/display_scale") 
    rospy.loginfo("Lancement du service display_scale")
    rospy.wait_for_service("/tourelle/image_ar/ellipse_correction") 
    rospy.loginfo("Lancement du service ellipse_correction")
    rospy.wait_for_service("/tourelle/image_ar/highter_behind") 
    rospy.loginfo("Lancement du service highter_behind")
    rospy.wait_for_service("/tourelle/image_ar/display_cloud_label")
    rospy.loginfo("Lancement du service display_cloud_label")
    rospy.wait_for_service("/tourelle/image_ar/merge_doses")
    rospy.loginfo("Lancement du service merge_doses")
     
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
      
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
