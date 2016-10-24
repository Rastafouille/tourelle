#! /usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = "Julien Favrichon, Jérémy Seyssaud"
__copyright__ = "Copyright 2015, CEA"
__license__ = ""
__version__ = "1.0.7"
__status__ = "Prototype"

import rospy
#from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

from tourelle_ros.msg import DoAcqAction,DoAcqFeedback, DoAcqResult, DoAcqGoal


def tourelle_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (DoAcqAction) to the constructor.
    client = actionlib.SimpleActionClient('tourelle', DoAcqAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = DoAcqGoal(pan_min=0,pan_max=180,tilt_min=85,tilt_max=85,step=5,speed=0.1,acq_angle=0,wait_by_step=1.0,overlap=0,conti=False)
  
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('tourelle_client_py')
        result = tourelle_client()
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
        print "Result: ",result
    except rospy.ROSInterruptException:
        #print("program interrupted before completion", file=sys.stderr)
        print "program interrupted before completion"
