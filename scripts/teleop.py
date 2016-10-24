#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 26 22:02:10 2016

@author: jf205732
"""

import rospy

from std_msgs.msg import Bool

import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to Bool
---------------------------
Space bar : start acquisition mode
Another keys : stop acquisition mode

CTRL-C to quit
"""

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

keybBindings = {
		' ':True
	       }
        

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_acq', Bool,queue_size=1)
    rospy.init_node('teleop_tourelle')
    status = 0
    try:
        print msg
        while(1):
            key = getKey()
            if key in keybBindings.keys():
                acq = keybBindings[key]
                print "Acquisition start!"
                """if (status == 14):
                    print msg"""
            else:
                if acq == False: print msg              
                acq = False
                print "Acquisition stop!"
                if (key == '\x03'):
                    break

            pub.publish(Bool(acq))

    except:
        print "erreur"

    finally:
        pub.publish(Bool(False))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


