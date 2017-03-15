#!/usr/bin/env python


#import roslib
#coppied and modified from tutorial:
import numpy as np
import pylab
import math
import random
from numpy.linalg import inv
import rospy
#import std_msgs.msg
#from camera_motor.msg import Prediction
from Prediction import Prediction
from std_msgs.msg import String
from std_msgs.msg import Time
#from std_msgs.msg import Prediction
import time

def talker():
    x_vel = 100232.4123
    y_vel = 1234.16759
    msg_sent_time = "%f" % rospy.Time.from_sec(time.time()).to_sec()  #infloat
    time_to_impact = 343.4232

    our_msg=str(x_vel)+ ','+str(y_vel)+','+str(msg_sent_time)+','+str(time_to_impact)
    pub =rospy.Publisher('camera_to_motor', String ,queue_size=10)
    rospy.init_node('talker', anonymous=True)

    #rate = rospy.Rate(10) #10 hz
    while not rospy.is_shutdown():

        rospy.loginfo("Here: "+our_msg)
        pub.publish(our_msg)
        #pub.publish(our_msg)
        rospy.sleep(1.0);
        #rate.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("had rospy exception")
        pass
