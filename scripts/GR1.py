#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import hid
import time
import rospkg
from datetime import datetime
from pytz import timezone
import rospy
import numpy as np
import matplotlib.pyplot as plt
from tourelle_ros.msg import GR1_msg
#from tourelle_ros.srv import spectro_clear
from std_srvs.srv import Empty,EmptyResponse
from tourelle_ros.srv import SpectroSave,SpectroSaveResponse

zulu_fmt = "%Y-%m-%dT%H:%M:%SZ"

class GR1():
    def __init__(self):
        self.USB_VENDOR_ID=rospy.get_param("/tourelle/GR1_node/GR1_VENDOR_ID",0x2a5a) 
        self.USB_PRODUCT_ID=rospy.get_param("/tourelle/GR1_node/GR1_PRODUCT_ID",0x0050) 
        self.COUNTRATE_INTERVAL=rospy.get_param("/tourelle/GR1_node/GR1_COUNTRATE_INTERVAL",1) #en secondes
        self.GR1_K=rospy.get_param("/tourelle/GR1_node/GR1_K") #etalonnage H*10 en microSv/h / cps 
        self.GR1_A=rospy.get_param("/tourelle/GR1_node/GR1_A") #etalonage en energie energie(kev) = A * canal + B
        self.GR1_B=rospy.get_param("/tourelle/GR1_node/GR1_B") #etalonage en energie energie(kev) = A * canal + B
        print "Creation du service spectro clear"
        self.service=rospy.Service("/tourelle/GR1_node/clear",Empty,self.cb_service_clear)    #déclaration du service  
        print "Creation du service spectro save .txt"
        self.service=rospy.Service("/tourelle/GR1_node/saveTXT",SpectroSave,self.cb_service_savetxt)
 	print "Creation du service spectro save .tka"
        self.service=rospy.Service("/tourelle/GR1_node/saveTKA",SpectroSave,self.cb_service_savetka)
        
        #GR10        
        #        self.USB_VENDOR_ID = 0x04d8
        #        self.USB_PRODUCT_ID = 0x0000
        #        self.COUNTRATE_INTERVAL = 20
        
        #GR05
        self.USB_VENDOR_ID = 0x2a5a
        self.USB_PRODUCT_ID = 0x0050
        #        self.COUNTRATE_INTERVAL = 1
        
        self.pub_GR1 = rospy.Publisher('/tourelle/GR1', GR1_msg, queue_size=10) 
        self.d=GR1_msg()
        self.d.header.frame_id='GR1_link'   
        self.d.header.stamp=rospy.Time.now()  
        
        rospack = rospkg.RosPack()
        self.pkgpath=rospack.get_path('tourelle_ros')
        
        usbPathList = []
        # Enumarate HID devices
        for d in hid.enumerate(0, 0):
            keys = d.keys()
            keys.sort()
        if d["vendor_id"] == self.USB_VENDOR_ID:
           usbPathList.append(d["path"])        
        self.devicePath = usbPathList
        print "Available Kromek devices =", self.devicePath
        print self.USB_PRODUCT_ID, self.USB_VENDOR_ID
         
        self.hidDevice = hid.device(self.USB_VENDOR_ID,self.USB_PRODUCT_ID)
        
        # Initialize counters        
        self.counts=np.zeros(4096) 
        self.totalcounter = 0 # keep track of total counts since start
        self.temps_ref=rospy.Time.now()

        self.x=np.zeros(4096) 
        for i in range(4096):
            self.x[i]= self.GR1_A*i+self.GR1_B
        #self.affichage()

        self.time_begin = time.time()  # for simu
        
    def Process(self):
        
        #remise a zero counter
        #self.counts=np.zeros(4096) 
        self.totalcounter = 0

        # Start timers
        start_time = time.time()
        while (time.time() - start_time) < self.COUNTRATE_INTERVAL:
            d = self.hidDevice.read(62)# timeout_ms = 50)
            if d:
                self.totalcounter += 1
                #print("impact")
                channel = (d[1]*256+d[2])/16 # ((d[1] << 8 | d[2]) >> 4) = 12bit channel
                self.counts[channel] += 1
            time.sleep(0.0001) # force yield for other threads

        self.d.spectro=self.counts
        deltat=rospy.Time.now()-self.d.header.stamp
        self.d.count_rate=float(self.totalcounter)/deltat.to_sec()
        self.d.dose_rate=(self.d.count_rate*self.GR1_K)
        self.d.header.stamp = rospy.Time.now()
        self.pub_GR1.publish(self.d)
        self.temps_acq=rospy.Time.now()-self.temps_ref
        
    def cb_service_clear(self,req):
        print "Spectro CLEAR SERVICE"
        self.counts=np.zeros(4096) 
        self.totalcounter = 0        
        self.temps_acq=0
        self.temps_ref=rospy.Time.now()   
        return EmptyResponse()

    def cb_service_savetxt(self,req):
        print req.filepath
        self.export2TXT(req.filepath) 
        print "Spectro SAVE .txt"
        return   SpectroSaveResponse()   
        
    def cb_service_savetka(self,req):
        print req.filepath
        self.export2TKA(req.filepath) 
        print "Spectro SAVE .tka"
        return   SpectroSaveResponse()  

    def export2TXT(self,path):
	x=self.x
	counts=self.counts        
        buf="timestamp : %s\n" % (datetime.now(timezone('UTC')).strftime(zulu_fmt))
        buf+="USB_VENDOR_ID : %s\n" % self.USB_VENDOR_ID
        buf+="USB_PRODUCT_ID : %s\n" % self.USB_PRODUCT_ID
        buf+="Measurement time : %0.3fs\n" % self.temps_acq.to_sec()
        buf+="Total count : %dcoups\n" % sum(self.counts)
        buf+="Coefficient etalonnage H*10 = %0.3f microSv/h/cps\n" %  self.GR1_K
        buf+="Coefficients etalonnage en energie,  energie (kev) =  %0.3f * canal + %0.3f\n" % (self.GR1_A,self.GR1_B)
	buf+="Canal\tEnergie\tcomptage\n"	
	for i in xrange(4096):
            buf+=str(i)+"\t"+str(x[i])+"\t"+str(counts[i])+"\n"
	with open(path,'w') as speFile :
	    speFile.write(buf)

    def export2TKA(self,path):
        speFile = open(path, "w")
        buf="%d\n" % self.temps_acq.to_sec()
        buf+="0\n" 
        for i in range(4094):
            buf+="%d\n" % (self.counts[i+2])
	speFile.write (buf)	
	speFile.close()
        
    def affichage(self):
        plt.clf()         
        plt.ylabel('spectre de la sonde GR1')
        plt.xlabel("channels")
        plt.title('Comptage: %.2f cps, DeD H*10: %.2f Sv/h, duree:%.1f s'%(self.d.count_rate,self.d.dose_rate,self.temps_acq.to_sec()))
        plt.plot(self.x,[(self.counts[i] ) for i in range(4096)],marker='v',color='r')                 
        plt.show()
        plt.pause(0.0001)
        
    def __del__(self):
        self.hidDevice.close()
        print "Connexion GR1 fermee"
    

if __name__ == '__main__':

    rospy.loginfo("Starting GR1")
    rospy.init_node("GR1_node")
    rate=rospy.Rate(1)    
    GR1 = GR1()
    #rospy.wait_for_service("/tourelle/GR1_node/clear")
    
    while not rospy.is_shutdown():
        GR1.Process()
        rate.sleep()


    rospy.loginfo("GR1 Node terminated")
 
    rospy.delete_param('~GR1_VENDOR_ID')
    rospy.delete_param('~GR1_PRODUCT_ID')
    rospy.delete_param('~GR1_COUNTRATE_INTERVAL')
    rospy.delete_param('~GR1_COUNTRATE_MAX')
    rospy.delete_param('~GR1_K')
    rospy.delete_param('~GR1_B')
    rospy.delete_param('~GR1_A')
  


  
