#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 04 16:36:16 2014
Test acquisition pano
"""
__author__ = "Julien Favrichon, Jérémy Seyssaud"
__copyright__ = "Copyright 2014, CEA"
__license__ = ""
__version__ = "1.0.7" #24/02/2015
__status__ = "Prototype"

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from tourelle_ros.srv import Acq,AcqResponse,AcqRequest,Acq_tele,Acq_teleResponse,Acq_teleRequest,Acq_tele2,Acq_tele2Response,Acq_tele2Request

from dynamixel_controllers.srv import TorqueEnable
  
if __name__=="__main__":
    
    """
    rospy.wait_for_service('/tourelle/get_acq_tele2')
    try:
        
        # Test pano service
        my_tele2srv = rospy.ServiceProxy("/tourelle/get_acq_tele2",Acq_tele2)
        my_tele2=my_tele2srv(pan_min=120.0,pan_max=240.0,tilt_min=85.0,tilt_max=90.0,step=1.0,speed=0.1,conti=True)
        #my_tele2 = my_tele2srv.call() # Appel du service avec les parametres par défaut
       
    except rospy.ServiceException, e:
        print "Service get_acq_tele2 call failed: %s"%e
    else:
        print "my_tele2srv OK"
    
    """
     # ************** Test pano service **************
    bridge=CvBridge()
    rospy.wait_for_service('/tourelle/get_pano')
    try:
        
       
        my_pano_srv = rospy.ServiceProxy("/tourelle/get_pano",Acq)
        my_pano = my_pano_srv(pan_min=60,pan_max=240,tilt_min=85,tilt_max=85,acq_angle=65,overlap=40)
        #my_pano = my_pano_srv.call() # Appel du service avec les parametres par défaut
        
        cv_pano = bridge.imgmsg_to_cv2(my_pano.pano, "bgr8")
                
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    else:
        cv2.imshow("Image window", cv_pano)
        cv2.waitKey(0)
        
        cv2.destroyAllWindows()
        
    
    #RELACHER LES MOTEURS
    TE_pan=rospy.ServiceProxy("/tourelle/j_BASE_ACC_position_controller/torque_enable",TorqueEnable)
    TE_pan(0)
    TE_tilt=rospy.ServiceProxy("/tourelle/j_ACC_PLT_position_controller/torque_enable",TorqueEnable)
    TE_tilt(0) 