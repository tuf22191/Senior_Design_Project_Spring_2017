#!/usr/bin/env python



#coppied and modified from tutorial:
import numpy as np
import pylab
import math
import random
from numpy.linalg import inv
import rospy
from std_msgs.msg import String
from std_msgs.msg import Time 
import time

def talker():
	#pub =rospy.Publisher('chatter',String ,queue_size=10)
	pub =rospy.Publisher('chatter',Time ,queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) #10 hz
	while not rospy.is_shutdown():
		hello_str =  rospy.Time.from_sec(time.time()) # http://docs.ros.org/diamondback/api/rospy/html/rospy.rostime.Time-class.html
		#hello_str = "Hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__=='__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		rospy.loginfo("had rospy exception")
		pass
