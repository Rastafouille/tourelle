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

#sudo hciconfig hci0 reset

import rospy
from time import sleep
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable, SetSpeed
from geometry_msgs.msg import Twist, Pose
import tf



class pilotage_nunchuck:
    def __init__(self):
        self.INIT_PAN_CONS = 0
        self.INIT_TILT_CONS = 0
        self.pan_cons_rad = 0
        self.tilt_cons_rad = 0        
        #self.load_max=0
        
        
        
        self.sub_pose=rospy.Subscriber("/tourelle/nunchuk/cmd_vel",Twist,self.cb_cmdvel)        

        self.pub_tilt=rospy.Publisher("/tourelle/j_ACC_PLT_position_controller/command",Float64,queue_size=10)
        self.pub_pan=rospy.Publisher("/tourelle/j_BASE_ACC_position_controller/command",Float64,queue_size=10)
        self.sub_tilt=rospy.Subscriber("/tourelle/j_ACC_PLT_position_controller/state",JointState, self.cb_tilt)
        self.sub_pan=rospy.Subscriber("/tourelle/j_BASE_ACC_position_controller/state",JointState, self.cb_pan)
        
        self.TILT_MIN=float(rospy.get_param("/tourelle/j_ACC_PLT_position_controller/motor/min"))/4096*2*3.14
        self.TILT_MAX=float(rospy.get_param("/tourelle/j_ACC_PLT_position_controller/motor/max"))/4096*2*3.14
        self.PAN_MIN=float(rospy.get_param("/tourelle/j_BASE_ACC_position_controller/motor/min"))/4096*2*3.14
        self.PAN_MAX=float(rospy.get_param("/tourelle/j_BASE_ACC_position_controller/motor/max"))/4096*2*3.14 
        #print ('TILT_MIN=',self.TILT_MIN,'et TILT_MAX=',self.TILT_MAX,'et PAN_MIN=',self.PAN_MIN,'et PAN_MAX=',self.PAN_MAX,)
      
 #Service call dynamixe speed
        rospy.wait_for_service("/tourelle/j_ACC_PLT_position_controller/set_speed")
        rospy.wait_for_service("/tourelle/j_BASE_ACC_position_controller/set_speed")
        self.pan_speed_srv = rospy.ServiceProxy("/tourelle/j_BASE_ACC_position_controller/set_speed",SetSpeed)
        self.tilt_speed_srv = rospy.ServiceProxy("/tourelle/j_ACC_PLT_position_controller/set_speed",SetSpeed)
        
        self.PAN_SPEED=rospy.get_param("/tourelle/pilotage_nunchuck/PAN_SPEED",0.05)
        self.TILT_SPEED=rospy.get_param("/tourelle/pilotage_nunchuck/TILT_SPEED",0.05)
        self.PAS=rospy.get_param("/tourelle/pilotage_nunchuck/PAS",0.02) 
    
        self.pan_speed_srv(self.PAN_SPEED)
        self.tilt_speed_srv(self.TILT_SPEED)
        
        self.tilt_cmd_rad=Float64()
        self.tilt_cmd_rad.data=0. #
        self.pan_cmd_rad=Float64()
        self.pan_cmd_rad.data=0.
        
        self.tilt_state_rad=999
        self.pan_state_rad=999
        
        self.pan_error_rad=0
        self.tilt_error_rad=0
        
        self.listener = tf.TransformListener()
        
        while self.tilt_state_rad==999 or self.pan_state_rad==999:
            sleep (0.01)
        

     
    def ok(self):
        pass
      #sleep (0.001)
      
      
    def cb_cmdvel(self,msg):
        if msg.angular.z>0.2 and self.pan_cons_rad>(self.PAN_MIN+(self.PAS*2)):
            self.pan_cmd_rad.data=self.pan_cons_rad-self.PAS
            self.pan_cons_rad=self.pan_cons_rad-self.PAS
            self.pub_pan.publish(self.pan_cmd_rad)
            #print ('PAN=',self.pan_cmd_rad.data,'et TILT=',self.tilt_cmd_rad.data)

        if msg.angular.z<-0.2 and self.pan_cons_rad<(self.PAN_MAX-(self.PAS*2)):
            self.pan_cmd_rad.data=self.pan_cons_rad+self.PAS
            self.pan_cons_rad=self.pan_cons_rad+self.PAS            
            self.pub_pan.publish(self.pan_cmd_rad)
            #print ('PAN=',self.pan_cmd_rad.data,'et TILT=',self.tilt_cmd_rad.data)

        if msg.linear.x<-0.2 and self.tilt_cons_rad<(self.TILT_MAX-(self.PAS*2)):
            self.tilt_cmd_rad.data=self.tilt_cons_rad+self.PAS
            self.tilt_cons_rad=self.tilt_cons_rad+self.PAS
            self.pub_tilt.publish(self.tilt_cmd_rad)
            #print ('PAN=',self.pan_cmd_rad.data,'et TILT=',self.tilt_cmd_rad.data)

        if msg.linear.x>0.2 and self.tilt_cons_rad>(self.TILT_MIN+(self.PAS*2)):
            self.tilt_cmd_rad.data=self.tilt_cons_rad-self.PAS            
            self.tilt_cons_rad=self.tilt_cons_rad-self.PAS
            self.pub_tilt.publish(self.tilt_cmd_rad)
            
        #print ('PAN=',self.pan_cmd_rad.data,'et TILT=',self.tilt_cmd_rad.data)
        print ('PAN consigne= %0.1f° et TILT Consigne= %0.1f°' % (self.pan_cons_rad*360/6.28,self.tilt_cons_rad*360/6.28))
        #while (abs(self.tilt_error_rad)>0.05)|(abs(self.pan_error_rad)>0.05) :
        #sleep(0.0001)
             
    def cb_pan(self,msg):
        if self.INIT_PAN_CONS :
            self.pan_error_rad=msg.error
            self.pan_state_rad=msg.current_pos
        #print (", pan state=",self.pan_state_rad)
        else:
            self.INIT_PAN_CONS=1
            self.pan_error_rad=msg.error
            self.pan_state_rad=msg.current_pos
            self.pan_cons_rad = self.pan_state_rad
     
#        if abs(msg.load)>self.load_max:
#             self.load_max=abs(msg.load)
#        print "loas max :",self.load_max
            
    def cb_tilt(self,msg):
        if self.INIT_TILT_CONS :        
            self.tilt_error_rad=msg.error
            self.tilt_state_rad=msg.current_pos
        else:
            self.INIT_TILT_CONS=1
            self.tilt_error_rad=msg.error
            self.tilt_state_rad=msg.current_pos
            self.tilt_cons_rad = self.tilt_state_rad
        
#        if abs(msg.load)>self.load_max:
#             self.load_max=abs(msg.load)
#        print "loas max :",self.load_max
#            
        
    def __delete__(self):
        print("pilotage nunchuck class destructor")
       
   
if __name__=="__main__":

    print "Starting Pilotage Nunchuck"
    rospy.init_node("pilotage_nunchuck")
    my_pilotage=pilotage_nunchuck()
    rate=rospy.Rate(100)
    while not rospy.is_shutdown():
             my_pilotage.ok()
             rate.sleep()
    
    rospy.loginfo("Pilotage tourelle Node terminated")
     
    rospy.delete_param("/tourelle/pilotage_nunchuck/PAN_SPEED")
    rospy.delete_param("/tourelle/pilotage_nunchuck/TILT_SPEED")
    rospy.delete_param("/tourelle/pilotage_nunchuck/PAS")

    
    
