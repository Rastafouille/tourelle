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
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from tourelle_ros.srv import Acq,AcqResponse,AcqRequest
from dynamixel_controllers.srv import TorqueEnable

class ImageSensorZone:

    def __init__(self, radius=10):
        self.image_pub = rospy.Publisher("/tourelle/image_dose",Image)
        self.aof_h = 64.5 # horizontal angle of field (deg)
        self.aof_v = 38.3 # verical angle of field (deg)
        self.grid_h = grid_h
        self.grid_v = grid_v
        self.color = (0,255,0)
        self.thickness = 2
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/tourelle/usb_cam_node/image_raw",Image,self.cb_im)
        
        self.dose_pcl = rospy.Subscriber("/pcl2",Image,self.cb_dose)
        self.radius_deg = radius
        self.radius = 1
        self.im_h = 1
        self.im_w = 1
    
    def XYZ2Px(self):
        pass
    
    @staticmethod
    def drawCircle(img,x,y,radius,color):
        cv2.circle(img, (x,y), radius, color, -1)
            
    def _getRadius(self):
        self.radius = (self.radius_deg / 2) * self.im_h / self.aof_h
        return self.radius
        
    def cb_dose(self,data):
        self.radius = self._getRadius()

    def cb_im(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
          print e
        (rows,cols,channels) = cv_image.shape
        self.im_h = rows
        self.im_w = cols
                  
        scale_deg_px = float(cols /self.aof_h)
        
        self.radius = self._getRadius(self.radius_deg)        
        
        #print "scale_deg_px",scale_deg_px
        center_x = cols / 2
        center_y = rows / 2
        
        #print "center_x",center_x
        #print "center_y",center_y
        
        cv2.circle(cv_image, (int(np.rint(center_x)),int(np.rint(center_y))),3, [255,0,127],-1)
        
        demi_delta_x = int(np.rint(0.5 * scale_deg_px * self.grid_v))
        
        indice_right = center_x + demi_delta_x    
        indice_left = center_x - demi_delta_x
        
        # 1ere lignes autour du centre
        if indice_right < cols :
            pt1 = (indice_right,0)
            pt2 = (indice_right,rows-1)
            cv2.line(cv_image,pt1,pt2,self.color,self.thickness)
        
        if indice_left >= 0 :
            pt1 = (indice_left,0)
            pt2 = (indice_left,rows-1)
            cv2.line(cv_image,pt1,pt2,self.color,self.thickness)
        
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
                cv2.line(cv_image,pt1,pt2,self.color,self.thickness)
                
            if indice_left >= 0 :
                pt1 = (indice_left,0)
                pt2 = (indice_left,rows-1)
                cv2.line(cv_image,pt1,pt2,self.color,self.thickness)
        
        # 1ere lignes autour du centre 
        demi_delta_y = int(np.rint(0.5 * scale_deg_px * self.grid_h))
        
        indice_hight = center_y + demi_delta_y  
        indice_low = center_y - demi_delta_y
        
        if indice_low >= 0 :
            pt1 = (0,indice_low)
            pt2 = (cols-1,indice_low)
            cv2.line(cv_image,pt1,pt2,self.color,self.thickness)
                
        if indice_hight < rows :
            pt1 = (0,indice_hight)
            pt2 = (cols-1,indice_hight)
            cv2.line(cv_image,pt1,pt2,self.color,self.thickness)
        
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
                cv2.line(cv_image,pt1,pt2,self.color,self.thickness)
                
            if indice_hight < rows :
                pt1 = (0,indice_hight)
                pt2 = (cols-1,indice_hight)
                cv2.line(cv_image,pt1,pt2,self.color,self.thickness)
    
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError, e:
          print e
      
def main(args):
    
    rospy.init_node('image_dose', anonymous=True)
    ic = ImageSensorZone(radius=10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
      
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)