#!/usr/bin/env python
import numpy as np
import pylab
import math
import random
from numpy.linalg import inv
import rospy
from std_msgs.msg import String
from std_msgs.msg import Time
import time

counter = 200
counter2 = 0
sum = 0.0
deltaT = 0.0

def callback(data):
        # 	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        #rospy.loginfo("I heard %s , %s", data.data, data.data.nsecs)
        py_time = rospy.Time.from_sec(time.time())
        deltaT = (py_time-data.data).to_nsec()/1000000000.0
# 	rospy.loginfo("Talker:%s.%s  Listener:%s.%s", data.data.secs, data.data.nsecs, py_time.secs, py_time.nsecs) 
 	rospy.loginfo("%s+", deltaT)              	
# 	rospy.loginfo("Talker:%s  Listener:%s  Diff(secs):%s ", data.data,  py_time, deltaT)              	
 #       counter2 = counter-1
#	counter = counter2
        #sum = sum +deltaT
	#if(counter ==0):
	#	rospy.loginfo("\n\n Avg. = %s", sum/200.0)
	#	counter = 200
	#	sum = 0.0 

def listener():
	rospy.init_node('listener',anonymous=True)
	rospy.Subscriber("chatter",Time, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
