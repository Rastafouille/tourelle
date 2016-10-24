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

from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud, Range, Image
from geometry_msgs.msg import Point32, PointStamped
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable

from cv_bridge import CvBridge, CvBridgeError
from tourelle_ros.srv import Acq,AcqResponse,AcqRequest

import numpy as np

from lib_tourelle.tourelle import panoAcqPara
from lib_tourelle.sensorsresults import ResultCamera
from lib_tourelle.consts import *

import tf
import math
import sys
import os

import subprocess
import cv2
import time
from time import sleep


def pic2Pano(img_files,path2stitcher="./stitching"):
    """
    Do a panorama of images by stitching pictures \n
    img_files: list of images to stitch \n
    return img
    """
    fileslist=""
    for img_file in img_files:
        fileslist +=" " + img_file
    #print fileslist
    #command = "\"./stitching --try_use_gpu yes" + fileslist + "\""
    #command = "\"./stitching " + fileslist + "\""
    #print command    
    #os.system(command)
    
    #command = "/home/jf205732/catkin_ws/src/tourelle_ros/scripts/stitching"
    command = path2stitcher
    (filepath,filename)=os.path.split(img_files[0])
    filepath_out = filepath + "/pano.jpeg"
    flags = " --output " + filepath_out
    print "Command=",command + fileslist + flags
    try:
        #retcode = subprocess.Popen([command, fileslist], shell=True, stdin=None, stdout=True, stderr=True, close_fds=True)
        #retcode = subprocess.call(command + fileslist + flags, shell=True)
        
        process = subprocess.Popen(command + fileslist + flags, shell=True, stdout=None,stdin=True, stderr=None, close_fds=True)
        process.wait()
        retcode = process.returncode
        print retcode
          
    except OSError as e:
        print >> sys.stderr, "Execution failed:", e
        return None
        
    else:
        if retcode < 0:
            print >>sys.stderr, "Child was terminated by signal", -retcode
        else:
            print >>sys.stderr, "Child returned", retcode
            
        return cv2.imread(filepath_out)

class CameraROS(object):
    """
    Class for camera sensor
    """
    def __init__(self,cam_topic="/tourelle/usb_cam_node/image_raw"):
        #Subscriber
        self.sub_cam = rospy.Subscriber(cam_topic, Image, self.cb_cam,queue_size = 1)   
        
        #Publisher
        #self.pub_image_grid = rospy.Publisher("/tourelle/image_grid",Image,queue_size=1)
        
        self.bridge = CvBridge()
            
        self.frame_width=0
        self.frame_height=0
        
        print("Camera device connected")
                       
        self.enable=True
        sleep(1)
     
    def __str__(self):
        return "Sensor type Camera, device num {0}".format(self.cam_topic)  
    
    def cb_cam(self,msg):
        """
        [sensor_msgs/Image]:
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        uint32 height
        uint32 width
        string encoding
        uint8 is_bigendian
        uint32 step
        uint8[] data
        """
        self.image = msg
        self.frame_width = msg.width
        self.frame_height = msg.height
                
    def acqOne(self):
        try:
            #cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")
            cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            #### direct conversion to CV2 ####
            #np_arr = np.fromstring(self.image, np.uint8)
            #cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            print("snap!")
            #print "cv_image",cv_image
            return cv_image
        except:
            print("Can't snap")
            return None
 
class TelemeterROS(object):
    '''
    Class for telemeter sensor
    '''
    def __init__(self,tele_topic="/tourelle/telemeter"):
        
        self.listener = tf.TransformListener()

        self.tele_point = PointStamped()
        self.tele_point.header.stamp = rospy.Time.now()
        self.tele_point.header.frame_id = 'laser_link'
        self.tele_point.point.x = 0
        self.tele_point.point.y = 0
        self.tele_point.point.z = 0

        self.base_point=PointStamped()
        self.base_point.header.stamp=rospy.Time.now()
        self.base_point.header.frame_id = 'base_link'
        
        self.pcl = PointCloud()
        self.pcl.header.stamp = rospy.Time.now()
        self.pcl.header.frame_id = "base_link"
        
        #Subscriber
        self.sub_telemeter = rospy.Subscriber(tele_topic,Range, self.cb_telemeter)
        
        self.pub_pcl=rospy.Publisher("/tourelle/pcl_acqui",PointCloud,queue_size=10)
        
        self.tele_topic = tele_topic
        self.enable = True
        
                
    def __str__(self):
        return "Sensor type telemetre, device port {0}".format(self.tele_topic)
    
    def cb_telemeter(self,msg):
        if (msg.range > (self.tele_point.point.x + 0.01)) | (msg.range < (self.tele_point.point.x - 0.01)) :
            if msg.range!=0:            
                self.tele_point.header.stamp = msg.header.stamp
                self.tele_point.point.x = msg.range
                
    def acqOne(self):
        try :
            self.listener.waitForTransform("base_link", "laser_link", self.tele_point.header.stamp, rospy.Duration(0.5))
            self.base_point = self.listener.transformPoint("base_link",self.tele_point)
            self.base_point.header.stamp = self.tele_point.header.stamp
            self.pcl.points.append(Point32(self.base_point.point.x,self.base_point.point.y,self.base_point.point.z))
            self.pcl.header.stamp = self.tele_point.header.stamp
            self.pub_pcl.publish(self.pcl)
        except:
            print "ERROR pcl acquisition"
            pass
        else:
            return self.pcl.points
        
    def acqConti(self, time_acq=0):
        '''
        Continius acquisition
        time : time of acquisition, 0 (default): infinite
        '''
        pass

    def __del__(self):
        #Release telemeter device
       
        print("Telemeter class destructor")
        
#Publisher pano
pub_pano=rospy.Publisher("/tourelle/pano/image_raw",Image,queue_size=1)

class tourelle_acquisition:
    def __init__(self):
        
        self.listener = tf.TransformListener()

        self.tele_point = PointStamped()
        self.tele_point.header.stamp = rospy.Time.now()
        self.tele_point.header.frame_id = 'laser_link'
        self.tele_point.point.x = 0
        self.tele_point.point.y = 0
        self.tele_point.point.z = 0

        self.base_point=PointStamped()
        self.base_point.header.stamp=rospy.Time.now()
        self.base_point.header.frame_id = 'base_link'
        
        self.pcl = PointCloud()
        self.pcl.header.stamp = rospy.Time.now()
        self.pcl.header.frame_id = "base_link"
        
        
        #Publisher
        self.pub_tilt=rospy.Publisher("/tourelle/j_ACC_PLT_position_controller/command",Float64,queue_size=10)
        self.pub_pan=rospy.Publisher("/tourelle/j_BASE_ACC_position_controller/command",Float64,queue_size=10)
        self.pub_pcl=rospy.Publisher("/tourelle/pcl_acqui",PointCloud,queue_size=10)
                
        #Subscriber
        self.sub_telemeter=rospy.Subscriber("/tourelle/telemeter",Range, self.cb_telemeter)
        self.sub_tilt=rospy.Subscriber("/tourelle/j_ACC_PLT_position_controller/state",JointState, self.cb_tilt)
        self.sub_pan=rospy.Subscriber("/tourelle/j_BASE_ACC_position_controller/state",JointState, self.cb_pan)
        
        #Camera
        self.my_cam = CameraROS(cam_topic="/tourelle/usb_cam_node/image_raw")
        self.stitch_file_name="pano"
                
                       
        self.ACQUI_TIME_STEP=rospy.get_param("ACQUI_TIME_STEP",0.5) # en secondes
        self.ACQUI_PAN_COUNT=rospy.get_param("ACQUI_PAN_COUNT",10) 
        self.ACQUI_TILT_COUNT=rospy.get_param("ACQUI_TILT_COUNT",10)
        self.ACQUI_PAN_MIN=rospy.get_param("ACQUI_PAN_MIN",120.0) #en degres
        self.ACQUI_PAN_MAX=rospy.get_param("ACQUI_PAN_MAX",200.0) #en degres
        self.ACQUI_TILT_MIN=rospy.get_param("ACQUI_TILT_MIN",40.0) #en degres
        self.ACQUI_TILT_MAX=rospy.get_param("ACQUI_TILT_MAX",60.0) #en degres     
        
        self.pan_min=rospy.get_param("pan_min",120.)
        self.pan_max=rospy.get_param("pan_max",240.)
        self.tilt_min=rospy.get_param("tilt_min",85.)
        self.tilt_max=rospy.get_param("tilt_max",85.)
        self.acq_angle=rospy.get_param("acq_angle",65.)
        self.overlap=rospy.get_param("overlap",60.)
        self.path2stitcher=rospy.get_param("path2stitcher","~/catkin_ws/src/tourelle_ros/scripts/stitching")
                     
        self.tilt_rad=Float64()
        self.tilt_rad.data=0. #
        self.pan_rad=Float64()
        self.pan_rad.data=0.
        
        self.pan_curr_rad = Float64()
        self.tilt_curr_rad = Float64()
        
        self.pan_error_rad = 0
        self.tilt_error_rad = 0
        
        self.pan_step=0
        self.tilt_step=0

        #Creation du service
        print "Creation du service"
        #pan_min=90, pan_max=240, tilt_min=90, tilt_max=90, acq_angle=60.0, overlap=5
        self.service=rospy.Service("/tourelle/get_pano",Acq,self.cb_service)    #déclaration du service 
        self.pano_image = Image()

        sleep(2)
        print 'OK'

    def cb_service(self,req):
        #self.setDxlPositionSI(req.pan,req.tilt)
        #self.go(req.tilt,req.pan)
        bridge = CvBridge()

        #Si l'ensemble des parametres d'entrees pan et tilt sont égales à 0 ont prend les valeurs par defaut
        if req.pan_min == 0.0 and req.pan_min == 0.0 and req.tilt_min == 0.0 and req.tilt_max == 0.0:
            pan_min = self.pan_min
            pan_max = self.pan_max
            tilt_min = self.tilt_min
            tilt_max = self.tilt_max
            acq_angle = self.acq_angle
            overlap = self.overlap
        else:
            pan_min = req.pan_min
            pan_max = req.pan_max
            tilt_min = req.tilt_min
            tilt_max = req.tilt_max
            acq_angle = req.acq_angle
            overlap = req.overlap
 
        try:
            acq_sequence,(pan_min_angle_real,pan_max_angle_real),(tilt_min_angle_real,tilt_max_angle_real) = panoAcqPara(pan_min=pan_min, pan_max=pan_max, tilt_min=tilt_min, tilt_max=tilt_max, acq_angle=acq_angle, step=None, overlap=overlap, begin_at_origine=False, matrix=True)    
        except:
            print "Error tilt or pan limit reach in acq_sequence"
            print "tilt min=",TILT_LIMIT_DOWN * 360/4096        
            print "tilt max=",TILT_LIMIT_UP * 360/4096
            print "pan min=",PAN_LIMIT_DOWN * 360/4096        
            print "pan max=",PAN_LIMIT_UP * 360/4096
        else:
            print "acq_sequence",acq_sequence
            print "(pan_min_angle_real,pan_max_angle_real)",(pan_min_angle_real,pan_max_angle_real)
            print "(tilt_min_angle_real,tilt_max_angle_real)",(tilt_min_angle_real,tilt_max_angle_real)
            print "acq_angle",acq_angle
            print "overlap",overlap
                 
        # Instantiation of results classes
        my_folder = os.getcwd() +''
        my_cam_results = ResultCamera(folder=my_folder,mission="Rosetta")
        
        # **** Acquisition
        self.startAcq(self.my_cam,my_cam_results,time_step=0,acq_matrix=acq_sequence,windscreen_wiper=True)
        
        # Creation du panorama
        """
        # pano_core, pano_py
        options = pano.Options()
        options.camera = camera
        options.stitch_size = pcv.Size(4000,2000)
        options.directory = pano_dir
        #options.image_names.assign(image_names)
        params = options.fitter_params
        params.error_thresh = 6;
        params.inliers_thresh = 15;
        params.maxiters = 100;
        params.nNeeded = 2;
        options.stitch_output = self.stitch_file_name
        """
        print "getAcqPos",my_cam_results.getAcqPos()
        #TODO construction de la grille d'acquisition en fonction des points d'acquisition du telemetre et ou sonde 
        #appliquer les tf pour passer en xy
        
        my_pano = pic2Pano(img_files = my_cam_results.getResults(),path2stitcher=self.path2stitcher)
        
        """
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)        
        """
        try:
            self.pano_image = bridge.cv2_to_imgmsg(my_pano, encoding="bgr8")
        except CvBridgeError, e:
            print(e)
        
        self.pano_image.header.stamp = rospy.Time.now()
        
        return AcqResponse(self.pano_image)

    def startCycle(self):
        # en degres et seconde
        self.stop=0
        pair=0
        self.pan_step=(self.ACQUI_PAN_MAX-self.ACQUI_PAN_MIN)/self.ACQUI_PAN_COUNT
        self.tilt_step=(self.ACQUI_TILT_MAX-self.ACQUI_TILT_MIN)/self.ACQUI_TILT_COUNT
        while self.stop==0:
            for i in range(0,self.ACQUI_TILT_COUNT+1):
                if pair == 0:
                    for j in range(0,self.ACQUI_PAN_COUNT+1):
                        if not rospy.is_shutdown():                        
                            self.go_wait((self.ACQUI_PAN_MIN+j*self.pan_step),(self.ACQUI_TILT_MIN+i*self.tilt_step))                       
                            self.pcl_acqui()                            
                            sleep(self.ACQUI_TIME_STEP)
                        else :
                            return
                    pair = 1
                else:
                    for j in range(self.ACQUI_PAN_COUNT+1,0,-1):
                        if not rospy.is_shutdown():                        
                            self.go_wait((self.ACQUI_PAN_MIN+j*self.pan_step),(self.ACQUI_TILT_MIN+i*self.tilt_step))    
                            self.pcl_acqui()                            
                            sleep(self.ACQUI_TIME_STEP)
                        else :
                            return
                    pair = 0
                self.stop=1

    def startAcq(self,sensor,sensor_results,acq_matrix,time_step=0,windscreen_wiper=True):
            """
            Start an acquisition sequence \n
            sensor: device object \n
            sensor_results : device results object \n
            acq_matrix: matrix acquisition sequence\n
            time_step - time step in seconde, if 0 then use acqOne device acquistion function \n
            windscreen_wiper: reversed pan sequence each odd/even line (default True) \n
            """
            odd = 1
            i = 0
            #sensor_results.addResult(data=sensor.acqOne(),time=None, position=(999,999), index=999)
            for raw in acq_matrix:
                print "raw", raw
                for pos in raw[::odd]:
                    print "pos",pos
                    a,b = pos
                    self.go_wait(a,b)
                    #self.tourelle_chain.wait_stopped([self.id_pan,self.id_tilt])
                    sleep(0.5)
                    pos_real = self.pan_curr_rad * 360. / (2. * math.pi), self.tilt_curr_rad * 360. / (2. * math.pi)
                    print "Position reelle pan/tilt", pos_real                
                                           
                    sleep(time_step)
                    if time_step != 0: #acqConti
                        sensor.acqConti(time_acq=time_step)
                    else: #acqOne
                        print "acqOne at pos",pos
                        sleep(2) #wait 2 sec for camera autofocus                   
                        #my_data=sensor.acqOne()
                        #print my_data
                        #cv2.imshow('image',my_data)
                        
                        sensor_results.addResult(data=sensor.acqOne(),current_time=time.localtime(), position=pos_real, acq_index=i)
                        i = i + 1
                if windscreen_wiper : odd = odd * (-1)
                
    # PAS ATTENTE POSITION ATTEINTE
    def setDxlPosition(self,pan_deg,tilt_deg):
        #publi les valeurs en rad a partir de commande en dégrés        
        self.pan_rad.data = pan_deg * 2 * math.pi / 360
        self.tilt_rad.data = tilt_deg * 2 * math.pi / 360
        self.pub_tilt.publish(self.tilt_rad)
        self.pub_pan.publish(self.pan_rad)
        print (', pan=',pan_deg,'et tilt=',tilt_deg)
    
    #ATTENTE DE LA POSITION EN REGARDANT "ERROR"
    def go_wait(self,pan_deg,tilt_deg):
        #publi les valeurs en rad a partir de commande en dégres et attend jusque etre à la position  
        self.pan_rad.data = pan_deg * 2 * math.pi / 360
        print self.pan_rad.data
        self.tilt_rad.data = tilt_deg * 2 * math.pi / 360
        self.pub_tilt.publish(self.tilt_rad)
        self.pub_pan.publish(self.pan_rad)
        print (', pan=',pan_deg,'et tilt=',tilt_deg)
        while (abs(self.tilt_error_rad)>0.05)|(abs(self.pan_error_rad)>0.05) :
            sleep(0.005)
            
    def cb_pan(self,msg):
        self.pan_curr_rad = msg.current_pos
        self.pan_error_rad = msg.error
        #print self.pan_error_rad

    def cb_tilt(self,msg):
        self.tilt_curr_rad = msg.current_pos
        self.tilt_error_rad = msg.error
     
    def pcl_acqui(self):
        try :
            self.listener.waitForTransform("base_link", "laser_link", self.tele_point.header.stamp, rospy.Duration(0.5))
            self.base_point=self.listener.transformPoint("base_link",self.tele_point)
            self.base_point.header.stamp=self.tele_point.header.stamp
            self.pcl.points.append(Point32(self.base_point.point.x,self.base_point.point.y,self.base_point.point.z))
            self.pcl.header.stamp = self.tele_point.header.stamp
            self.pub_pcl.publish(self.pcl)
        except:
            print "ERROR pcl acquisition"
            pass
        
    def cb_telemeter(self,msg):
        if (msg.range>(self.tele_point.point.x+0.01)) | (msg.range<(self.tele_point.point.x-0.01)) :
            if msg.range!=0:            
                self.tele_point.header.stamp=msg.header.stamp
                self.tele_point.point.x=msg.range
    
    def __delete__(self):
        print("Tourelle class destructor")

        
   
if __name__=="__main__":

    print "Starting Tourelle Acquisition"
    rospy.loginfo("Starting Tourelle Acquisition")
    rospy.init_node("tourelle_acquisition")
    
    my_tourelle_acquisition = tourelle_acquisition()
    rate=rospy.Rate(50)
    
    #Acquisition Telemeter
    my_tourelle_acquisition.startCycle()
    
    # Acquisition pano
    # Lancement du  service get_pano
    #rospy.wait_for_service("/tourelle/get_pano") 
    #rospy.loginfo("Lancement du service get_pano")
    
#    pub_pano.publish(my_tourelle_acquisition.pano_image)
    
    # Spin until ctrl + c
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        
    rospy.loginfo("Tourelle Acquisition Node terminated")
    
    #RELACHER LES MOTEURS
    TE_pan=rospy.ServiceProxy("/tourelle/j_BASE_ACC_position_controller/torque_enable",TorqueEnable)
    TE_pan(0)
    TE_tilt=rospy.ServiceProxy("/tourelle/j_ACC_PLT_position_controller/torque_enable",TorqueEnable)
    TE_tilt(0)
    rospy.delete_param("ACQUI_TIME_STEP")
    rospy.delete_param("ACQUI_PAN_COUNT")
    rospy.delete_param("ACQUI_TILT_COUNT")
    rospy.delete_param("ACQUI_PAN_MIN")
    rospy.delete_param("ACQUI_PAN_MAX")
    rospy.delete_param("ACQUI_TILT_MIN")
    rospy.delete_param("ACQUI_TILT_MAX")

