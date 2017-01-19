#!/usr/bin/env python



#coppied and modified from tutorial:
import numpy as np
import pylab
import math
import random
from numpy.linalg import inv
import rospy
from std_msgs.msg import String


def talker():
	pub =rospy.Publisher('chatter',String,queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) #10 hz
	while not rospy.is_shutdown():
		hello_str = "Hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__=='__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		rospy.loginfo("had rospy exception")
		pass
