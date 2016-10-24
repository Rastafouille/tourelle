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
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import SetSpeed

from consts import *

import math

from time import sleep
'''
from threading import Thread

class Afficheur(Thread):

    """Thread chargé simplement d'afficher une lettre dans la console."""

    def __init__(self, lettre):
        Thread.__init__(self)
        self.lettre = lettre

    def run(self):
        """Code à exécuter pendant l'exécution du thread."""
        i = 0
        while i < 20:
            sys.stdout.write(self.lettre)
            sys.stdout.flush()
            attente = 0.2
            attente += random.randint(1, 60) / 100
            time.sleep(attente)
            i += 1

# Création des threads
thread_1 = Afficheur("1")
thread_2 = Afficheur("2")

# Lancement des threads
thread_1.start()
thread_2.start()

# Attend que les threads se terminent
thread_1.join()
thread_2.join()
'''

class TourelleROS(object):
    def __init__(self):
    
        #Publisher
        self.pub_tilt=rospy.Publisher("/tourelle/j_ACC_PLT_position_controller/command",Float64,queue_size=10)
        self.pub_pan=rospy.Publisher("/tourelle/j_BASE_ACC_position_controller/command",Float64,queue_size=10)
                
        #Subscriber
        self.sub_tilt=rospy.Subscriber("/tourelle/j_ACC_PLT_position_controller/state",JointState, self.cb_tilt)
        self.sub_pan=rospy.Subscriber("/tourelle/j_BASE_ACC_position_controller/state",JointState, self.cb_pan)
       
        #Service call dynamixe speed
        rospy.wait_for_service("/tourelle/j_ACC_PLT_position_controller/set_speed")
        rospy.wait_for_service("/tourelle/j_BASE_ACC_position_controller/set_speed")
        self.pan_speed_srv = rospy.ServiceProxy("/tourelle/j_BASE_ACC_position_controller/set_speed",SetSpeed)
        self.tilt_speed_srv = rospy.ServiceProxy("/tourelle/j_ACC_PLT_position_controller/set_speed",SetSpeed)
        
        self.default_pan_speed,self.default_tilt_speed = self.getDxlSpeed()
                             
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

        print 'Tourelle Init OK'
    
    def startAcq(self,sensor,sensor_results,acq_matrix,wait_by_step=0.5,speed=None,conti=False,windscreen_wiper=True):
        """
        Start an acquisition sequence \n
        sensor: device object \n
        sensor_results : device results object \n
        acq_matrix: matrix acquisition sequence\n
        wait_by_step : wait time in each step in seconde,
        speed: if not 0 then run the acquisition sequence at speed value without step and pause\n
        conti: if True use acqConti device acquisition function else (default) use acqOne\n
        windscreen_wiper: reversed pan sequence each odd/even line (default True) \n
        """
        odd = 1
        i = 0
        #Put the turret is at sequence initial position
        a,b = acq_matrix[0][0]
        self.goWaitwSpeed(a,b,self.default_pan_speed,self.default_tilt_speed)
        
        #sensor_results.addResult(data=sensor.acqOne(),time=None, position=(999,999), index=999)
        if conti: #TODO utilisation des threads pour acquisition data durant mouvement tourelle
            #acq_matrix = np.array(acq_matrix)
        
            #matrix initialisation
            acq_matrix_conti = [[0 for x in range(2)] for x in range(len(acq_matrix))] 
            #conti mode : keep only first and last item of each matrix raw
            for j in range(0,len(acq_matrix_conti),1):              
                acq_matrix_conti[j][0] = acq_matrix[j][0]
                acq_matrix_conti[j][1] = acq_matrix[j][-1]
                
            for raw in acq_matrix_conti:
                print "raw", raw
                for pos in raw[::odd]:
                    print "pos",pos
                    a,b = pos
                                                
                    if speed is None:speed=MOVING_SPEED_RADS
                        
                    self.goWaitwSpeed(a,b,speed,speed)
                
                    #sleep(0.5)
                    pos_real = self.getDxlPosition()
                    print "Position reelle pan/tilt", pos_real                
                    sleep(0.5)
                   
                    sensor_results.addResult(data=sensor.acqConti(time_acq=wait_by_step),current_time=time.localtime(), position=pos_real, acq_index=i)
                   
                if windscreen_wiper : odd = odd * (-1)
                    
        else:
            for raw in acq_matrix:
                print "raw", raw
                for pos in raw[::odd]:
                    print "pos",pos
                    a,b = pos
                    self.go_wait(a,b)
                    #self.tourelle_chain.wait_stopped([self.id_pan,self.id_tilt])
                    #sleep(0.5)
                    pos_real = self.getDxlPosition()
                    print "Position reelle pan/tilt", pos_real                
                    #sleep(0.5)
                    print "acqOne at pos",pos
                    sleep(wait_by_step) #wait 2 sec for camera autofocus      
                    
                    sensor_results.addResult(data=sensor.acqOne(),current_time=time.localtime(), position=pos_real, acq_index=i)
                    i = i + 1
                if windscreen_wiper : odd = odd * (-1)
        
        #Set default speed to motors after acquisition sequence
        self.setDxlSpeed(self.default_pan_speed,self.default_tilt_speed)
        
        return sensor_results
            
    def startSeq(self,acq_matrix,wait_by_step=0.5,speed=None,conti=False,windscreen_wiper=True):
        """
        Start a sequence \n
        acq_matrix: matrix acquisition sequence\n
        wait_by_step : wait time in each step in seconde,
        speed: if not 0 then run the acquisition sequence at speed value without step and pause\n
        conti: if True use acqConti device acquisition function else (default) use acqOne\n
        windscreen_wiper: reversed pan sequence each odd/even line (default True) \n
        """
        odd = 1
        i = 0
        #Put the turret is at sequence initial position
        a,b = acq_matrix[0][0]
        self.goWaitwSpeed(a,b,self.default_pan_speed,self.default_tilt_speed)
     
        if conti: #TODO utilisation des threads pour acquisition data durant mouvement tourelle
            #acq_matrix = np.array(acq_matrix)
        
            #matrix initialisation
            acq_matrix_conti = [[0 for x in range(2)] for x in range(len(acq_matrix))] 
            #conti mode : keep only first and last item of each matrix raw
            for j in range(0,len(acq_matrix_conti),1):              
                acq_matrix_conti[j][0] = acq_matrix[j][0]
                acq_matrix_conti[j][1] = acq_matrix[j][-1]
                
            for raw in acq_matrix_conti:
                print "raw", raw
                for pos in raw[::odd]:
                    print "pos",pos
                    a,b = pos
                                                
                    if speed is None:speed=MOVING_SPEED_RADS
                        
                    self.goWaitwSpeed(a,b,speed,speed)
                
                    sleep(0.5)
                    pos_real = self.getDxlPosition()
                    print "Position reelle pan/tilt", pos_real                
                    sleep(0.5)
                   
                    #sensor_results.addResult(data=sensor.acqConti(time_acq=wait_by_step),current_time=time.localtime(), position=pos_real, acq_index=i)
                   
                if windscreen_wiper : odd = odd * (-1)
                    
        else:
            for raw in acq_matrix:
                print "raw", raw
                for pos in raw[::odd]:
                    print "pos",pos
                    a,b = pos
                    self.go_wait(a,b)
                    #self.tourelle_chain.wait_stopped([self.id_pan,self.id_tilt])
                    sleep(0.5)
                    pos_real = self.getDxlPosition()
                    print "Position reelle pan/tilt", pos_real                
                    sleep(0.5)
                    print "acqOne at pos",pos
                    sleep(wait_by_step) #wait 2 sec for camera autofocus      
                    
                    #sensor_results.addResult(data=sensor.acqOne(),current_time=time.localtime(), position=pos_real, acq_index=i)
                    i = i + 1
                if windscreen_wiper : odd = odd * (-1)
        
        #Set default speed to motors after acquisition sequence
        self.setDxlSpeed(self.default_pan_speed,self.default_tilt_speed)
        
        return 0
                
    # PAS ATTENTE POSITION ATTEINTE
    def setDxlPosition(self,pan_deg,tilt_deg):
        #publi les valeurs en rad a partir de commande en dégrés        
        self.pan_rad.data = pan_deg * math.pi / 180
        self.tilt_rad.data = tilt_deg * math.pi / 180
        self.pub_tilt.publish(self.tilt_rad)
        self.pub_pan.publish(self.pan_rad)
        #print (', pan=',pan_deg,'et tilt=',tilt_deg)
        
    def setDxlPositionAndSpeed(self,pan_deg,tilt_deg,pan_speed=MOVING_SPEED_RADS,tilt_speed=MOVING_SPEED_RADS):
        self.setDxlSpeed(pan_speed,tilt_speed)
        self.setDxlPosition(pan_deg,tilt_deg)
        
    def getDxlPosition(self):
        #recupere valeurs en rad et retourne en deg
        return self.pan_curr_rad * 180./ math.pi, self.tilt_curr_rad * 180./ math.pi
    
    def setDxlSpeed(self,pan_speed,tilt_speed):
        #ros_pan_speed = Float64(pan_speed)
        #ros_tilt_speed = Float64(tilt_speed)
        ros_pan_speed = float(pan_speed)
        ros_tilt_speed = float(tilt_speed)
        self.pan_speed_srv(ros_pan_speed)
        self.tilt_speed_srv(ros_tilt_speed)
        print"Speed SET, pan_speed=%f , tilt_speed=%f"%(ros_pan_speed,ros_tilt_speed)
        
    def getDxlSpeed(self):
        #ros_pan_speed = Float64(pan_speed)
        #ros_tilt_speed = Float64(tilt_speed)
        pan_speed=rospy.get_param("/tourelle/j_BASE_ACC_position_controller/joint_speed",MOVING_SPEED_RADS)
        tilt_speed=rospy.get_param("/tourelle/j_ACC_PLT_position_controller/joint_speed",MOVING_SPEED_RADS)
        print"Speed GET, pan_speed=%f , tilt_speed=%f"%(pan_speed,tilt_speed)
        return (pan_speed,tilt_speed)        
                       
    #ATTENTE DE LA POSITION EN REGARDANT "ERROR"
    def go_wait(self,pan_deg,tilt_deg):
        #publi les valeurs en rad a partir de commande en dégres et attend jusque etre à la position
        self.setDxlPosition(pan_deg,tilt_deg)
        #precision en radian des moteurs: 0,001533578
        while (abs(self.tilt_error_rad) > 0.05) or (abs(self.pan_error_rad) > 0.05) :
            sleep(0.005)
        
    def goWaitwSpeed(self,pan_deg,tilt_deg,pan_speed=MOVING_SPEED_RADS,tilt_speed=MOVING_SPEED_RADS):
        #publi les valeurs en rad a partir de commande en dégres et attend jusque etre à la position
        print "set position pan=",pan_deg
        print "set position tilt=",tilt_deg
        self.setDxlPositionAndSpeed(pan_deg,tilt_deg,pan_speed=pan_speed,tilt_speed=tilt_speed)
        #precision en radian des moteurs: 0,001533578
        sleep(2) #tempo pour laisser le temps à la tourelle de quitter la zone d'incertitude des moteurs
        while (abs(self.tilt_error_rad) > 0.05) or (abs(self.pan_error_rad) > 0.05) :
            #print "tilt_error_rad",self.tilt_error_rad
            #print "pan_error_rad",self.pan_error_rad
            sleep(0.005)
            
    def cb_pan(self,msg):
        self.pan_curr_rad = msg.current_pos
        self.pan_error_rad = msg.error

    def cb_tilt(self,msg):
        self.tilt_curr_rad = msg.current_pos
        self.tilt_error_rad = msg.error
        
    def __delete__(self):
        print("Tourelle class destructor")
        
if __name__ == "__main__": 
    print "TourelleROS Classes"