#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = "Julien Favrichon, Jérémy Seyssaud"
__copyright__ = "Copyright 2015, CEA"
__license__ = ""
__version__ = "1.0.7"
__status__ = "Prototype"

import rospy

import actionlib

from time import sleep

from tourelle_ros.msg import DoAcqAction,DoAcqFeedback,DoAcqResult

from lib_tourelle.tourelle import panoAcqPara
from lib_tourelle.tourelle_ros import TourelleROS
from lib_tourelle.consts import *

class TourelleAction(TourelleROS):
    # create messages that are used to publish feedback/result
    _feedback = DoAcqFeedback()
    _result = DoAcqResult()

    def __init__(self, name):
        super(TourelleAction, self).__init__()
                
        self.pan_min=rospy.get_param("pan_min",0.)
        self.pan_max=rospy.get_param("pan_max",300.)
        self.tilt_min=rospy.get_param("tilt_min",85.)
        self.tilt_max=rospy.get_param("tilt_max",85.)
        self.acq_angle=rospy.get_param("acq_angle",65.)
        self.overlap=rospy.get_param("overlap",60.)        
        
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, DoAcqAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        
        self.debug = 0
            
    def getDxlPosition(self):
        return super(TourelleAction, self).getDxlPosition()
        
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        #Si l'ensemble des parametres d'entrees pan et tilt sont égales à 0 ont prend les valeurs par defaut
        if goal.pan_min == 0.0 and goal.pan_min == 0.0 and goal.tilt_min == 0.0 and goal.tilt_max == 0.0:
            pan_min = self.pan_min
            pan_max = self.pan_max
            tilt_min = self.tilt_min
            tilt_max = self.tilt_max
            overlap = self.overlap
            step = 5.0
            conti = False
            speed = 5.0
            acq_angle = 0
            wait_by_step = 0.5
            
        else:
            pan_min = goal.pan_min
            pan_max = goal.pan_max
            tilt_min = goal.tilt_min
            tilt_max = goal.tilt_max
            overlap = goal.overlap
            step = goal.step
            speed = goal.speed
            conti = goal.conti
            acq_angle = goal.acq_angle
            wait_by_step = goal.wait_by_step
                
        if step == 0:
            step = 1 
            
        try:
            acq_sequence,(pan_min_angle_real,pan_max_angle_real),(tilt_min_angle_real,tilt_max_angle_real) = panoAcqPara(pan_min=pan_min, pan_max=pan_max, tilt_min=tilt_min, tilt_max=tilt_max, acq_angle=acq_angle, step=step, overlap=overlap, begin_at_origine=False, matrix=True) 
        except:
            print "Error tilt or pan limit reach in acq_sequence"
            print "tilt min=",TILT_LIMIT_DOWN * 360/4096        
            print "tilt max=",TILT_LIMIT_UP * 360/4096
            print "pan min=",PAN_LIMIT_DOWN * 360/4096        
            print "pan max=",PAN_LIMIT_UP * 360/4096
        else:
            if self.debug > 0 : print "acq_sequence",acq_sequence
            print "(pan_min_angle_real,pan_max_angle_real)",(pan_min_angle_real,pan_max_angle_real)
            print "(tilt_min_angle_real,tilt_max_angle_real)",(tilt_min_angle_real,tilt_max_angle_real)
            print "step",step
            print "conti",conti
            print "speed",speed
            print "speed",acq_angle
            print "speed",wait_by_step           
               
        # **** # start executing the action : Sequence
        """
        Start a sequence \n
        acq_matrix: matrix acquisition sequence\n
        wait_by_step : wait time in each step in seconde,
        speed: if not 0 then run the acquisition sequence at speed value without step and pause\n
        conti: if True use acqConti device acquisition function else (default) use acqOne\n
        windscreen_wiper: reversed pan sequence each odd/even line (default True) \n
        """
        acq_matrix = acq_sequence
        windscreen_wiper =  True
        odd = 1
        i = 1
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
            print "len(acq_matrix_conti)",len(acq_matrix_conti)
            print "len(acq_matrix_conti[0])",len(acq_matrix_conti[0]) 
            len_acq_matrix = len(acq_matrix_conti) * 2
            for raw in acq_matrix_conti:
                if self.debug > 0 : print "raw", raw
                for pos in raw[::odd]:
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        #self._result.total_steps_achieves = i
                        #self._as.set_preempted(result=self._result)
                        self._as.set_preempted()
                        success = False
                        break
                    print "pos",pos
                    a,b = pos
                                                
                    if speed is None:speed=MOVING_SPEED_RADS
                        
                    self.goWaitwSpeed(a,b,speed,speed)
                    
                    self._feedback.percent_complete = float(i * 100 / len_acq_matrix) #avancement sequence par ligne
                    self._as.publish_feedback(self._feedback)
                    i = i+1
                    # publish info to the console for the user
                    rospy.loginfo('%s: Executing, sequence running, %f complete' % (self._action_name, self._feedback.percent_complete))
                    
                    pos_real = self.getDxlPosition()
                    
                    print "Position reelle pan/tilt", pos_real                
                    sleep(0.1)
                   
                    #sensor_results.addResult(data=sensor.acqConti(time_acq=wait_by_step),current_time=time.localtime(), position=pos_real, acq_index=i)
                if self._as.is_active() == False:
                    success = False                    
                    break
                
                if windscreen_wiper : odd = odd * (-1)
                success = True
                    
        else:
            if self.debug > 0 : print "len(acq_matrix)",len(acq_matrix)
            if self.debug > 0 : print "len(acq_matrix[0])",len(acq_matrix[0])
            len_acq_matrix = len(acq_matrix) * len(acq_matrix[0])
            for raw in acq_matrix:
                if self.debug > 0 : print "raw", raw
                for pos in raw[::odd]:
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        #self._result.total_steps_achieves = i
                        #self._as.set_preempted(result=self._result)
                        self._as.set_preempted()
                        success = False
                        break
                    print "pos",pos
                    a,b = pos
                    self.go_wait(a,b)
                    #self.tourelle_chain.wait_stopped([self.id_pan,self.id_tilt])
                    pos_real = self.getDxlPosition()
                    if self.debug > 0 : print "Position reelle pan/tilt", pos_real                
                    if self.debug > 0 : print "acqOne at pos",pos
                    sleep(wait_by_step)    
                    self._feedback.percent_complete = float(i * 100 / len_acq_matrix) #sequence
                    self._as.publish_feedback(self._feedback)
                     
                    # publish info to the console for the user
                    rospy.loginfo('%s: Executing, sequence running, %f complete'% (self._action_name, self._feedback.percent_complete))
                    
                    #sensor_results.addResult(data=sensor.acqOne(),current_time=time.localtime(), position=pos_real, acq_index=i)
                    i = i + 1
                if self._as.is_active() == False:
                    success = False                 
                    break                
                if windscreen_wiper : odd = odd * (-1)
                success = True
                
        #Set default speed to motors after acquisition sequence
        self.setDxlSpeed(self.default_pan_speed,self.default_tilt_speed)
               
        if success:
            self._result.total_steps_achieves = i
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('tourelle')
    server = TourelleAction(rospy.get_name())
    rospy.spin()
