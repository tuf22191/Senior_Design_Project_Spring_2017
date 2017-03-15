#!/usr/bin/env python
import numpy as np
import pylab
import math
import random
from numpy.linalg import inv
import rospy
from std_msgs.msg import String
from std_msgs.msg import Time
#from Prediction import Prediction



def callback(data):

    x =str(data.data).split(',')
    #y = [float(i) for i in x]
    print x
    print '%f' % float(x[0])
    print '%f' % float(x[1])
    print '%f' % float(x[2])
    print '%f' % float(x[3])


def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber('camera_to_motor', String , callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
