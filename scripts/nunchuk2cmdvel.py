#!/usr/bin/env python
# -*- coding: utf-8 -*-

#sur le PC H@ri a chaque branchement de clé bluetooth "sudo hciconfig hci0 reset"

import serial
from math import sqrt
import matplotlib.pyplot as plt
import numpy as np 
import cwiid
import time
import rospy
from threading import Lock
from geometry_msgs.msg import Twist
import os

if __name__=="__main__":
    led = 0
    rpt_mode = 0
    rumble = 0
    mesg = False
    global wiimote
    t=Twist()
    rospy.init_node("nunchuk2cmdvel")
    pub_cmd_vel=rospy.Publisher("/nunchuk/cmd_vel",Twist,queue_size=1)
    print "AVANT DE LANCER LE NOEUD POUR LA PREMIERE FOIS : sudo hciconfig hci0 reset"

    try:
        print 'Press 1 + 2 on your Wii Remote now to connect it...'
        wiimote = cwiid.Wiimote()
        ComWiimote=1
        print 'Wiimote connected !!!'
        wiimote.rumble = 1
        print 'niveau batterie sur 208 : ',wiimote.state['battery']
        print 'Press PLUS and MINUS together to disconnect and quit.\n'
        time.sleep(0.5)
        wiimote.rumble = 0
    except RuntimeError:
        print 'Wiimote connexion failed ...'
        ComWiimote =0
    
    Exit=False
    while not rospy.is_shutdown() and Exit==False:
                time.sleep(0.01)
                # lecture wiimote            
                wiimote.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_ACC | cwiid.RPT_EXT
                WiiState=wiimote.state
                #print 'Wii State =',WiiState
                #print 'Nunchuck',wiimote.state['nunchuk']['buttons'],wiimote.state['nunchuk']['stick'][0],wiimote.state['nunchuk']['stick'][1]  
                t.linear.x=float ((wiimote.state['nunchuk']['stick'][1]-128))/128
                t.angular.z=float ((wiimote.state['nunchuk']['stick'][0]-128))/128
                if t.linear.x<0.1 and t.linear.x>-0.1:
                    t.linear.x=0
                if t.angular.z<0.1 and t.angular.z>-0.1:
                    t.angular.z=0
               # print "Publication", t
                pub_cmd_vel.publish(t)
            
                # Detection + et - pour exit     
                if (wiimote.state['buttons'] - cwiid.BTN_PLUS - cwiid.BTN_MINUS == 0):
                    wiimote.rumble = 1
                    time.sleep(0.5)
                    wiimote.rumble = 0
                    Exit=True
    wiimote.close()
    print "Node terminated"
