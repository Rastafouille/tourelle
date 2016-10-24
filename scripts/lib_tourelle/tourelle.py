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

import time # on peux appeler sleep par tim.sleep
from time import sleep # on peut directement utiliser sleep sans le time.
from dxl.dxlchain import * # importe toute methode du module dxlchain dans le dossier dxl

#from cloud_plot import *
import json
import os
import random
import numpy as np
import subprocess

from consts import *
from sensors import *
from sensorsresults import *


def frange(start, stop, step):
    r = start
    while r <= stop:
        yield r
        r += step

def pic2Pano(img_files):
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
    command = "./stitching"
    (filepath,filename)=os.path.split(img_files[0])
    flags = " --output " + filepath + "/pano.jpeg"
    print "Command=",command + fileslist + flags
    try:
        retcode = subprocess.call(command + fileslist + flags, shell=True)
        if retcode < 0:
            print >>sys.stderr, "Child was terminated by signal", -retcode
        else:
            print >>sys.stderr, "Child returned", retcode
    except OSError as e:
        print >>sys.stderr, "Execution failed:", e

def panoAcqPara(pan_min, pan_max, tilt_min, tilt_max, acq_angle, step=None, overlap=0.0, begin_at_origine= False, matrix = True):
    """
    Function to define the acquisition parameters \n 
    pan_min: pan_min in degree \n        
    pan_max: pan_max in degree \n
    tilt_min: tilt_min in degree \n
    tilt_max: tilt_max in degree \n
    acq_angle: solid angle covered by acquisition, degrees \n
    step: acquisition step in degree. If not None, matrix acquisition is define with this value (default None) \n
    overlap: overlap between two acquisitions in percent (default 0) \n
    begin_at_origine: if true, begin the acquisition at pan_min and tilt_min, otherwise the acquisition taking to account acq_angle/2 to begin acquisition (default False) \n
    matrix : if true return sequence in a 2 dimensionals python list else return 2 1 dimensional python lists (delault true) \n
    
    return: acq_sequence,(pan_min_angle_real,pan_max_angle_real),(tilt_min_angle_real,tilt_max_angle_real) \n
    return: pan_acq_sequence,(pan_min_angle_real,pan_max_angle_real), tilt_acq_sequence,(tilt_min_angle_real,tilt_max_angle_real) \n
    pan_min_angle_real : pan_min recalculated with begin_at_origine parameters \n
    pan_max_angle_real : pan_max recalculated with acq_angle and overlap parameters\n
    tilt_min_angle_real : tilt_min recalculated with acq_angle and overlap parameters\n
    tilt_max_angle_real : tilt_max recalculated with acq_angle and overlap parameters\n
    """
    pan_acq_sequence=[]
    tilt_acq_sequence=[]
   
    """    
    f_pan = lambda n : n * acq_angle (1 - overlap /100 ) + acq_angle / 2
    s3 = [f_pan(n) for n in range(11)]
    """
       
    if step is None:  ##### No step defined ###########
        print "overlap", overlap
        print "1 - overlap/100",(1-float(overlap)/100)
    
        pan_i_max = pan_max - acq_angle/2
        tilt_j_max = tilt_max - acq_angle/2 
        
        if begin_at_origine:
            print "begin_at_origine",begin_at_origine
            '''
            pan_i = float(pan_min-acq_angle/2) 
            tilt_j = float(tilt_min-acq_angle/2)
            
            pan_min_angle_real = float(pan_min)-float(acq_angle/2)
            tilt_min_angle_real = float(tilt_min)-float(acq_angle/2)
            '''
            pan_i0 = pan_min + acq_angle/2
            tilt_j0 = tilt_min + acq_angle/2
            
            pan_min_angle_real = pan_min
            tilt_min_angle_real = tilt_min
        
                          
        else:
            print "begin_at_origine",begin_at_origine
            pan_i = float(pan_min) 
            tilt_j = float(tilt_min) 
            
            pan_i0 = pan_min
            tilt_j0 = tilt_min
            
            pan_min_angle_real = pan_min - acq_angle/2
            tilt_min_angle_real = tilt_min - acq_angle/2
            
        
        print "pan_i_max",pan_i_max
        print "tilt_j_max",tilt_j_max
        pan_i = pan_i0
        tilt_j = tilt_j0
        print "pan_i",pan_i
        print "tilt_j",tilt_j
        #pan_acq_sequence.append(pan_min)
        #tilt_acq_sequence.append(tilt_min)
        i=0
        j=0    
        #PAN
        if pan_i0 > pan_i_max : # if only one step for pan
            pan_acq_sequence.append(pan_i0)
        else:
            while pan_i < pan_i_max:
                #pan_i = pan_min + i * acq_angle * (1 - float(overlap)/100 ) + acq_angle / 2
                #pan_i = pan_min + i * (acq_angle / 2 - acq_angle * (1 - float(overlap)/100 ))
                pan_i = pan_i0 + i * acq_angle * (1 - float(overlap)/100 )
                print i
                print pan_i
                i=i+1
                pan_acq_sequence.append(pan_i)
    
        pan_max_angle_real = pan_acq_sequence[-1] + acq_angle/2
        print "pan_max_angle_real",pan_max_angle_real
        
        #TILT
        if tilt_j0 > tilt_j_max : # if only one step for tilt
            tilt_acq_sequence.append(tilt_j0)
        else:
            while tilt_j < tilt_j_max :
                #tilt_j = tilt_min + j * acq_angle * (1 - float(overlap)/100 ) + acq_angle / 2
                tilt_j = tilt_j0 + j * acq_angle * (1 - float(overlap)/100 )
                print j
                print tilt_j
                j=j+1
                tilt_acq_sequence.append(tilt_j)
    
        print "tilt_acq_sequence",tilt_acq_sequence
        tilt_max_angle_real = tilt_acq_sequence[-1] + acq_angle/2
        print "tilt_max_angle_real",tilt_max_angle_real
    
    else: ############# stef is define
        pan_min_angle_real = pan_min
        tilt_min_angle_real = tilt_min
        pan_max_angle_real = pan_max
        tilt_max_angle_real = tilt_max
        pan_i_max = pan_max
        tilt_j_max = tilt_max
        pan_i0 = pan_min
        tilt_j0 = tilt_min
        
        for i in frange(pan_min,pan_max,step):
            pan_acq_sequence.append(i)
        for j in frange(tilt_min,tilt_max,step):
            tilt_acq_sequence.append(j)
        
        """
        #range use integer only
        for i in range(pan_min,pan_max,step):
            pan_acq_sequence.append(i)
        for j in range(tilt_min,tilt_max,step):
            tilt_acq_sequence.append(j)
        """
    
    if matrix :
        # acq_sequence Matrix
        acq_sequence=[[0 for x in range(len(pan_acq_sequence))] for x in range(len(tilt_acq_sequence))] #matrix initialisation
        #acq_sequence = np.zeros((len(pan_acq_sequence),len(tilt_acq_sequence)))

        #print "matrix acq_sequence init",acq_sequence
        n=0
        m=0
        for tilt_acq in tilt_acq_sequence:
            for pan_acq in pan_acq_sequence: 
                acq_sequence[m][n]=(pan_acq,tilt_acq)
                n=n+1
            n=0
            m=m+1
            
        #print "matrix acq_sequence (pan,tilt)"
        #print('\n'.join([''.join(['{:4}'.format(item) for item in row]) for row in acq_sequence]))

        if pan_min < PAN_LIMIT_DOWN*360/4096 or pan_max > PAN_LIMIT_UP*360/4096 or tilt_min < TILT_LIMIT_DOWN*360/4096 or tilt_max > TILT_LIMIT_UP*360/4096  :
            raise
            return
        else:
            return acq_sequence,(pan_min_angle_real,pan_max_angle_real),(tilt_min_angle_real,tilt_max_angle_real)
    else:
        if pan_min < PAN_LIMIT_DOWN*360/4096 or pan_max > PAN_LIMIT_UP*360/4096 or tilt_min < TILT_LIMIT_DOWN*360/4096 or tilt_max > TILT_LIMIT_UP*360/4096  :
            raise
            return
        else:
            return pan_acq_sequence,(pan_min_angle_real,pan_max_angle_real), tilt_acq_sequence,(tilt_min_angle_real,tilt_max_angle_real)
    #pan_step=int((pan_max-pan_min)/pan_count)
    #tilt_step=int((tilt_max-tilt_min)/tilt_count)

class Tourelle(object):
    '''
    id_pan: Motor ID for the pan mouvement \n
    id_tilt: Motor ID for the tilt mouvement \n
    port_dxl:   \n
    '''
    def __init__(self,id_pan,id_tilt,port_dxl,simu=False):
        self.id_pan = id_pan
        self.id_tilt = id_tilt
        self.port_dxl = port_dxl
        self.rate_dxl = 1000000
        self.stop = 0
        self.sensors = [] # Table for mount sensor object
        if simu==False:
            self.initDxl()
        
    def __del__(self):
        #Release dxl
        try:
            self.tourelle_chain.close()
        except:
            print "Can't close dxl connection"
            
    def initDxl(self):
        try:
            self.tourelle_chain=DxlChain(self.port_dxl,self.rate_dxl)
        except:
            print("Can't connect to the DxlChain")
            #raise
        else:
            print("Connection to the DxlChain OK")
            self.tourelle_chain.get_configuration() # recupere les infos du servo, equivalent a ping broacast
            print(self.tourelle_chain.motors) # --> {10: <dxl.dxlmotors.DxlMotorAX12W object at 0xb7089bac>}
            self.tourelle_chain.set_reg(self.id_pan,"cw_angle_limit",PAN_LIMIT_DOWN)
            self.tourelle_chain.set_reg(self.id_pan,"ccw_angle_limit",PAN_LIMIT_UP)
            self.tourelle_chain.set_reg(self.id_pan,"p_gain",P_GAIN)
            self.tourelle_chain.set_reg(self.id_pan,"moving_speed",MOVING_SPEED)
            self.tourelle_chain.set_reg(self.id_tilt,"cw_angle_limit",TILT_LIMIT_DOWN)
            self.tourelle_chain.set_reg(self.id_tilt,"ccw_angle_limit",TILT_LIMIT_UP)
            self.tourelle_chain.set_reg(self.id_tilt,"p_gain",P_GAIN)
            self.tourelle_chain.set_reg(self.id_tilt,"moving_speed",MOVING_SPEED)
          
    def addSensor(self, sensor,pos_l=None,pos_w=None): #TODO
        '''
        Add a sensor to the turret \n
        sensor: Sensor object \n
        pos_l: length between sensor and rotation axe of the turret platform \n
        pos_w: width between sensor and rotation axe of the turret platform \n
        '''
        self.sensors.append((sensor,pos_l,pos_w))
        
    def listSensor(self): # TODO
        if len(self.sensors) > 0:      
            return self.sensors
        else:
            return None
            
    def acqPipe(self, para=False): #TODO
        '''
        Set sensors and parameter's for acquisition sequence \n
        para: parallelize acq, default False
        '''
        if len(self.sensors) > 0:
            for sensor in self.sensors:
                dev,l,x=sensor
                if dev.isEnable() == True:
                    res1 = dev.acqOne()
    
    def getDxlPosition(self):
        try:        
            pos=[self.tourelle_chain.get_reg(self.id_pan,"present_position"),self.tourelle_chain.get_reg(self.id_tilt,"present_position")]
        except:
            print "Dxl communication problem.Can't return position"
            pos=None
        
        return pos

    def getDxlPositionSI(self):
        try:
            pos=[self.tourelle_chain.get_reg(self.id_pan,"present_position")*360/4096,self.tourelle_chain.get_reg(self.id_tilt,"present_position")*360/4096]
        except:
            print "Dxl communication problem.Can't return position"
            pos=None
            
        return pos

    def setDxlPosition(self,a,b):
        """
        Set position input command to Dxl Motor \n
        a - pan mvt \n
        b - tilt mvt \n
        """
        #print "commande pan tilt=",a,b
        if (a < PAN_LIMIT_DOWN or a > PAN_LIMIT_UP):
            print "position Pan inexistante"
        elif (b < TILT_LIMIT_DOWN or b > TILT_LIMIT_UP):
             print "position Tilt inexistante"
        else:
            try:
                self.tourelle_chain.sync_write_pos([self.id_pan,self.id_tilt],[a,b])
                self.tourelle_chain.wait_stopped([self.id_pan,self.id_tilt])
            except:
                print "Dxl communication problem.Can't set position"

    def setDxlPositionSI(self,a,b):
        """
        Set position input command to Dxl Motor in degree \n
        a - pan mvt \n
        b - tilt mvt \n
        """
        #print "commande pan tilt=",a,b
        if (a < (360.*PAN_LIMIT_DOWN/4096.) or a > (360.*PAN_LIMIT_UP/4096.)):
            print "position Pan inexistante"
        elif (b < (360.*TILT_LIMIT_DOWN/4096.) or b > (360.*TILT_LIMIT_UP/4096.)):
            print "position Tilt inexistante"
        else:
            try:
                self.tourelle_chain.sync_write_pos([self.id_pan,self.id_tilt],[int(a*4096./360.),int(b*4096./360.)])
                #print "moving"
                self.tourelle_chain.wait_stopped([self.id_pan,self.id_tilt])
                #print "stop moving"
            except:
                print "Dxl communication problem.Can't set position"
                raise

    def startCycle(self,sensor,sensor_results,time_step,pan_count,tilt_count,pan_min,pan_max,tilt_min,tilt_max):
        """
        Start a tourelle moving sequence \n
        time_step - time step in seconde \n
        pan_count - \n
        tilt_count \n
        pan_min \n
        pan_max \n
        tilt_min \n
        tilt_max \n
        """
        # en degres et seconde
        self.stop=0
        pair=0
        pan_step=(pan_max-pan_min)/pan_count
        tilt_step=(tilt_max-tilt_min)/tilt_count
        print ('Steps calcul, pan_step=',pan_step,'et tilt_step=',tilt_step)
        while self.stop==0: # ne marche pas pour le moment in faudra mettre sur une touche l'arret du cycle
            for i in range(0,tilt_count+1):
                if pair == 0:
                    for j in range(0,pan_count+1):
                        self.setDxlPositionSI((pan_min+j*pan_step),(tilt_min+i*tilt_step))
                        self.tourelle_chain.wait_stopped([self.id_pan,self.id_tilt])
                        sleep(time_step)
                        sensor_results.addResult(data=sensor.acqOne(),current_time=time.localtime(), position=self.getDxlPositionSI(), acq_index=i)
                        print 'position:',self.getDxlPositionSI(),'distance:',sensor.acqOne()
                    pair = 1
                else:
                    for j in range(pan_count+1,0,-1):
                        self.setDxlPositionSI((pan_min+j*pan_step),(tilt_min+i*tilt_step))
                        self.tourelle_chain.wait_stopped([self.id_pan,self.id_tilt])
                        sleep(time_step)
                        sensor_results.addResult(data=sensor.acqOne(),current_time=time.localtime(), position=self.getDxlPositionSI(), acq_index=i)
                        print 'position:',self.getDxlPositionSI(),'distance:',sensor.acqOne()
                    pair = 0
                self.stop=1
        
    def startAcq(self,sensor,sensor_results,time_step,acq_matrix,windscreen_wiper=True):
        """
        Start an acquisition sequence \n
        sensor: device object \n
        sensor_results : device results object \n
        time_step - time step in seconde, if 0 then use acqOne device acquistion function \n
        acq_matrix: matrix acquisition sequence\n
        windscreen_wiper: reversed pan sequence each odd/even line (default True) \n
        """
        odd = 1
        i = 0
        #sensor_results.addResult(data=sensor.acqOne(),time=None, position=(999,999), index=999)
        for raw in acq_matrix:
            print "raw", raw
            for pos in raw[::odd]:
                print "pos",pos
                a,b=pos
                
                try:
                    self.setDxlPositionSI(a,b)
                    #self.tourelle_chain.wait_stopped([self.id_pan,self.id_tilt])
                    pos_real=self.getDxlPositionSI()
                    print "getDxlPositionSI()", pos_real                
                except:
                     print "Dxl communication problem"
                     pos_real= pos
                   
                sleep(time_step)
                if time_step != 0: #acqConti
                    sensor.acqConti(time_acq=time_step)
                else: #acqOne
                    print "acqOne at pos",pos
                    sleep(1) #wait 1 sec                    
                    #my_data=sensor.acqOne()
                    #print my_data
                    #cv2.imshow('image',my_data)
                    
                    sensor_results.addResult(data=sensor.acqOne(),current_time=time.localtime(), position=pos_real, acq_index=i)
                    i = i + 1
            if windscreen_wiper : odd = odd * (-1)
        
if __name__ == "__main__": 
    print "Tourelle Class"         