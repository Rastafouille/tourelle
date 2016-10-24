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

from sensor_msgs.msg import PointCloud2, Range, PointField
from geometry_msgs.msg import PointStamped
from tourelle_ros.msg import Dose_rate
  
class ImageAR:

    def __init__(self, grid_h=10, grid_v=10):
        self.image_pub = rospy.Publisher("/tourelle/image_grid",Image)
        self.aof_h = 64.5 # horizontal angle of field (deg)
        self.aof_v = 38.3 # verical angle of field (deg)
        self.grid_h = grid_h
        self.grid_v = grid_v
        self.rows = 0
        self.cols = 0
        self.color = (0,255,0)
        self.thickness = 1
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        # Suscribers
        self.image_sub = rospy.Subscriber("/tourelle/usb_cam_node/image_raw",Image,self.callback)
        self.sub_telemeter=rospy.Subscriber("/tourelle/telemeter",Range, self.cb_telemeter)
        self.sub_dose_rate = rospy.Subscriber("tourelle/dose_rate",Dose_rate, self.cb_dose_rate)         
            
        self.process_grid = True
        self.im_grid = None
        
    def setGrid(self,grid_h,grid_v):
        self.grid_h = grid_h
        self.grid_v = grid_v
        self.process_grid = True
        
    
    @staticmethod
    def imageBlend(img1,img2):

        # Create a ROI
        rows,cols,channels = img2.shape
        roi = img1[0:rows, 0:cols]
        
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
        img1[0:rows, 0:cols] = dst
        
        return img1
        
    def pcl2_acqui(self):
        try :
            self.listener.waitForTransform("base_link", "laser_link", self.telemeter_point.header.stamp, rospy.Duration(1))
            self.base_point=self.listener.transformPoint("base_link",self.telemeter_point)
            self.base_point.header.stamp=self.telemeter_point.header.stamp
            
            pts_new = np.array([float(self.base_point.point.x),float(self.base_point.point.y),float(self.base_point.point.z)])
            self.pts = np.vstack((self.pts,pts_new))
            
            r = int(255*self.ddd/self.ddd_max_range)
            g = int(255-255*self.ddd/self.ddd_max_range)
            b = 0
            colors_new = np.array([b,g,r])
            print colors_new 
            self.colors = np.vstack((self.colors,colors_new))
            
            mon_pcl2 = self.xyzrgb2pc(xyz=self.pts,bgr=self.colors,frame_id='base_link',time2use=self.telemeter_point.header.stamp)
            self.pub_pcl2.publish(mon_pcl2)            
            self.pcl2.header.stamp = self.telemeter_point.header.stamp
            self.pub_pcl2.publish(self.pcl2)
            #print self.pcl2
        except:
            print "ERROR pcl acquisition"
            pass        
        
        
    def cb_telemeter(self,msg):
        if (msg.range>(self.telemeter_point.point.x+0.01)) | (msg.range<(self.telemeter_point.point.x-0.01)) :
            if msg.range!=0:            
                self.telemeter_point.header.stamp=msg.header.stamp
                self.telemeter_point.point.x=msg.range
                self.pcl2_acqui()
    
    def buildGrid(self):
        
        cols = self.cols
        rows = self.rows
        
        if self.im_grid is None:
            size = rows, cols, 3
            self.im_grid = np.zeros(size, dtype=np.uint8)
            self.im_grid[:,:] = (0,0,0)
    
        scale_deg_px = float(cols /self.aof_h)
        #print "scale_deg_px",scale_deg_px
        center_x = self.cols / 2
        center_y = self.rows / 2
        
        #print "center_x",center_x
        #print "center_y",center_y
        
        cv2.circle(self.im_grid, (int(np.rint(center_x)),int(np.rint(center_y))),3, [255,0,127],-1)
        
        demi_delta_x = int(np.rint(0.5 * scale_deg_px * self.grid_v))
        
        indice_right = center_x + demi_delta_x    
        indice_left = center_x - demi_delta_x
        
        # 1ere lignes autour du centre
        if indice_right < cols :
            pt1 = (indice_right,0)
            pt2 = (indice_right,rows-1)
            cv2.line(self.im_grid,pt1,pt2,self.color,self.thickness)
        
        if indice_left >= 0 :
            pt1 = (indice_left,0)
            pt2 = (indice_left,rows-1)
            cv2.line(self.im_grid,pt1,pt2,self.color,self.thickness)
        
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
                cv2.line(self.im_grid,pt1,pt2,self.color,self.thickness)
                
            if indice_left >= 0 :
                pt1 = (indice_left,0)
                pt2 = (indice_left,rows-1)
                cv2.line(self.im_grid,pt1,pt2,self.color,self.thickness)
        
        # 1ere lignes autour du centre 
        demi_delta_y = int(np.rint(0.5 * scale_deg_px * self.grid_h))
        
        indice_hight = center_y + demi_delta_y  
        indice_low = center_y - demi_delta_y
        
        if indice_low >= 0 :
            pt1 = (0,indice_low)
            pt2 = (cols-1,indice_low)
            cv2.line(self.im_grid,pt1,pt2,self.color,self.thickness)
                
        if indice_hight < rows :
            pt1 = (0,indice_hight)
            pt2 = (cols-1,indice_hight)
            cv2.line(self.im_grid,pt1,pt2,self.color,self.thickness)
        
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
                cv2.line(self.im_grid,pt1,pt2,self.color,self.thickness)
                
            if indice_hight < rows :
                pt1 = (0,indice_hight)
                pt2 = (cols-1,indice_hight)
                cv2.line(self.im_grid,pt1,pt2,self.color,self.thickness)
    

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
          print e
        (self.rows,self.cols,channels) = cv_image.shape
        
        if self.process_grid:
            self.buildGrid()
            self.process_grid = False
        
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imageBlend(cv_image,self.im_grid), "bgr8"))
        except CvBridgeError, e:
          print e
      
def main(args):
    
    rospy.init_node('Image_AR', anonymous=True)
    iar = ImageAR(grid_h=5, grid_v=5)
    try:
        rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
      
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)