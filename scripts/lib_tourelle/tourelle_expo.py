#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Class to operate turret in ROS
"""
__author__ = "Julien Favrichon, Jérémy Seyssaud"
__copyright__ = "Copyright 2015, CEA"
__license__ = ""
__version__ = "1.0.0" #02/05/2015
__status__ = "Prototype"

#import tourelle
from consts import *
from sensors import *
from sensorsresults import *
from tourelle import *

#import roslib; roslib.load_manifest('tourelle_ros')
import rospy

#import time
from time import sleep
#import geometry_msgs.msg.Twist
from geometry_msgs.msg import Twist,Vector3
#import geometry_msgs.msg.Pose
from turtlesim.msg import Pose
from std_msgs.msg import String
import serial
from threading import Lock
from sensor_msgs.msg import Imu

#from std_srvs.srv import Empty
#from turtlesim.srv import TeleportAbsolute
#from tourelle_ros.srv import Goal,GoalResponse,GoalRequest
from tourelle_ros.srv import *

class TourelleROS(Tourelle):
    def __init__(self):
        #TODO : récupérer les parametres dans fichier parametre
        try:
            Tourelle.__init__(self,id_pan=1,id_tilt=21,port_dxl='/dev/ttyACM1',simu=False)
        except:
            raise Exception("Impossible de se connecter à la tourelle")
        else:
            #Creation du publisher de position
            #self.pub_message=rospy.Publisher("/tourelle/message",String,queue_size=10)
            print "Creation du publisher de position"
            self.pub_position=rospy.Publisher("/tourelle/position",Vector3,queue_size=1)
                    
            #sleep(1)
            #Creation du suscriber de pilotage de la tourelle -> plutot passer par un service
            print "Creation du subscriber de commande"
            self.sub_cmdpos=rospy.Subscriber("/tourelle/cmd_pos",Vector3,self.cb_cmdpos)
            
            sleep(2)
            #Creation du service
            print "Creation du service"
            self.service=rospy.Service("/tourelle/goal",Goal,self.cb_service)    #déclaration du service    
            
            self.goal_tilt =  None
            self.goal_pan = None
            
            self.done = False
            self.lock=Lock()
        
    def pubPosition(self):
        message_position = Vector3()        
        try:
            pos = self.getDxlPositionSI()
            message_position.x = 0.0
            message_position.y = float(pos[0])
            message_position.z = float(pos[1])
        except:
            print "Can't get position"
            
        self.pub_position.publish(message_position)

    def cb_service(self,req):
        self.setDxlPositionSI(req.pan,req.tilt)
        #self.go(req.tilt,req.pan)
        
        return GoalResponse()
    
    def set_goal(self,tilt,pan):
        self.done = False
        self.goal_tilt = tilt
        self.goal_pan = pan

    def wait_goal(self):
        while not self.done and not rospy.is_shutdown():
            sleep(0.1)
            
    def go(self,tilt,pan):
        self.set_goal(tilt,pan)
        
        self.wait_goal()    
         
    def cb_cmdpos(self,msg):
        """
        geometry_msgs/Vector3
          float64 x
          float64 y
          float64 z
        """
        pan = msg.y
        tilt = msg.z
        self.setDxlPositionSI(pan,tilt)
        
        self.pubPosition()
        
    def __delete__(self):
        print "Destruction de l'objet"

if __name__=="__main__":
    
    print "Starting"
    rospy.init_node("tourelle")
    sleep(1)
    try:
        ma_tourelle=TourelleROS()
    except:
        print "Impossible de creer l'objet Tourelle"
    else:
        while not rospy.is_shutdown(): #on boucle tant qu'il n'y a pas de demande de shutdown (rosnode kill, CTL+C)
            try:
                #print "Reception"
                ma_tourelle.pubPosition()
                #sleep(0.1)
            except:
                print "problème de reception"    
        
            
        #print t
        sleep(0.1) # permet d'attendre l'envoie du paquet        
        """
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        """
    finally:
        print "Node terminated"
        
#    print "Attente service /clear)"
#    rospy.wait_for_service("/clear")    
#    print "service dispo"    
#   
#    clear = rospy.ServiceProxy("/clear",Empty) #creation d'un proxy d'accès au service avec le nom du service et le type
#    print "Appel du service"    
#    clear() #appel du service    
    
    # Applel du service teleportation
#    rospy.wait_for_service("/turtle1/teleport_absolute") 
#    teleport = rospy.ServiceProxy("/turtle1/teleport_absolute",TeleportAbsolute)
#    teleport.call(6,2,5)    
    
    
#    turtlegoal.go(1,2)
#    turtlegoal.go(8,2)
#    turtlegoal.go(8,8)
#    turtlegoal.go(1,8)
#    turtlegoal.go(2,2)
   