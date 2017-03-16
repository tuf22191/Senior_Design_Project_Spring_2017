#!/usr/bin/env python


#import roslib
#coppied and modified from tutorial:
import cv2
import numpy as np
import flycapture2 as fc2
import time
import datetime
import sys
import imutils
import argparse
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


def talker():
    x_vel = 100232.4123
    y_vel = 1234.16759
    msg_sent_time = "%f" % rospy.Time.from_sec(time.time()).to_sec()  #infloat
    time_to_impact = 343.4232

    our_msg=str(x_vel)+ ','+str(y_vel)+','+str(msg_sent_time)+','+str(time_to_impact)
    pub =rospy.Publisher('camera_to_motor', String ,queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # to run program: press "up" arrow so it says "python ....."
    frInd = 0
    viz = True
    size_rank_of_target = 0

    # set up logfile
    cur_datetime = datetime.datetime.now()
    cur_dtime_str = cur_datetime.strftime('%Y-%m-%d_%Hh%Mm%Ss')
    logfname = '//arenatracker_%s.csv' % cur_dtime_str
    logf = open(logfname, 'w', 0) # 0 forces flush after each write call (buffer = 0)
    logf.write('framenum,UTCtime,fps,animx,animy,bbx,bby,bbw,bbh,bbtheta\n')


    # set up camera
    c = fc2.Context()
    c.connect(*c.get_camera_from_index(0))
    p = c.get_property(fc2.FRAME_RATE)
    c.set_property(**p)
    if viz:
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    print c.get_camera_info()
    print 'Camera setup complete'
    print "pixel formats"

    # start capture process
    c.start_capture()
    im = fc2.Image()
    start = time.time()
    progress_t = start
    first_start = start
    progress_interval = 10 # time between prints in seconds

    #other person's code
    firstFrame = None

    print "Starting tracking"
    print "...press control-c to stop."
    try:
        meters_per_pixel_ratio = None
        meters_per_pixel_ratio_x = None
        #BALLSIZE = .047 # 47mm diameter
        BALLSIZE= .14  #in meters
        BALLSIZE_X=.14 #in meters
        ratminx = None
        ratmaxx = None
        ratminy = None
        ratmaxy = None
        start_time_speed = None
        end_time_speed = None
        vertical_vel_ball = None
        horizontal_vel_ball = None
        deltay_mins = None #average the change in y_maxes with change in y_mins
        deltax_mins = None #average the change in x_maxes with change in x_mins
        finito = -4

        while True:
            zz =c.retrieve_buffer(im)
            img = np.array(zz) # these are coming in as crazy bayered things--and in grey
            #cv2.imshow('yellow',img)
            imm = img
            # print "OHO"
            # print (dir(zz))
            # print "get format is: ", zz.get_format()
            # print "OHO"
            timestamp = time.time()
            img = cv2.cvtColor(img, cv2.COLOR_BAYER_RG2RGB)# REal is COLOR_BAYER_BG2BGR) # debayer (pattern verified--cv2 likes BGR order, if vizing with cv2, use this)
            frame = img
            imm =img
            #added code here NOT OUR CODE! Took it from other person
            frame = imutils.resize(frame, width=500) #height = 800
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            # if the first frame is None, initialize i


            if firstFrame is None:
                firstFrame = gray; continue

            # compute the absolute difference between the current frame and
            # first frame
            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            # dilate the thresholded image to fill in holes, then find contours
            # on thresholded image
            thresh = cv2.dilate(thresh, None, iterations=2)
            (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    # loop over the contours
            x_min_value =0
            x_max_value =0
            y_min_value =0
            y_max_value =0
            x_top_intermediate =0
            y_top_intermediate =0
            counter_number = 0
            had_counter = False

            for count_ in cnts:

                # if the contour is too small, ignore it
                if cv2.contourArea(count_) < 500:#args["min_area"]
                    continue

                # compute the bounding box for the contour, draw it on the frame,
                # and update the text

                (x, y, w, h) = cv2.boundingRect(count_)
                ## INSERTED HERE for aggregation of the countor rectangles
                counter_number = counter_number+1
                x_top_intermediate = x+w
                y_top_intermediate = y+h
                if(counter_number==1):
                    x_min_value =x
                    x_max_value =x_top_intermediate
                    y_min_value =y
                    y_max_value =y_top_intermediate
                else:
                    if(x<x_min_value):
                        x_min_value = x
                    if(x_top_intermediate>x_max_value):
                        x_max_value = x_top_intermediate
                    if(y<y_min_value):
                        y_min_value = y
                    if(y_top_intermediate>y_max_value):
                        y_max_value = y_top_intermediate
                had_counter = True
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                ## INSERTED HERE for aggregation of the countor rectangles


             # after going through all of the object contours
            if(had_counter):
                if(start_time_speed!=None): #end the timer #high priority
                    end_time_speed = time.time()
                    deltay_mins = (abs(y_max_value-ratmaxy)+abs(y_min_value-ratminy))/2.0
                    deltax_mins = (abs(x_max_value-ratmaxx)+abs(x_min_value-ratminx))/2.0
                if(ratminx == None): #initialize the first frame values
                    ratminx = x_min_value
                    ratminy = y_min_value
                    ratmaxx = x_max_value
                    ratmaxy = y_max_value
                    start_time_speed = time.time()
                    meters_per_pixel_ratio  = BALLSIZE/(y_max_value-y_min_value+0.0)
                    meters_per_pixel_ratio_x  = BALLSIZE_X/(x_max_value-x_min_value+0.0)
                # only for drawing not that important
                cv2.rectangle(frame, (x_min_value, y_min_value), (x_max_value, y_max_value), (0, 255, 0), 2)
                text = "Occupied"
                cv2.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
                #print('Ball x_min_value: ' +str(x_min_value) +' x_max_value: ' + str(x_max_value)+' y_min_value: '+str(y_min_value)+' y_max_value: '+str(y_max_value))
                #only need to do y right now for vertical tests
                if(end_time_speed!=None):
                    #do the speed calculation here and
                    speed_ball_y = (deltay_mins * meters_per_pixel_ratio)/(end_time_speed-start_time_speed)
                    speed_ball_x = (deltax_mins * meters_per_pixel_ratio_x)/(end_time_speed-start_time_speed)
                    print ("X -SPEED is : " + str(speed_ball_x)+ " m/s \n")
                    print ("Y -SPEED is : " + str(speed_ball_y)+ " m/s \n\n")
                    our_msg=str(speed_ball_x)+ ','+str(speed_ball_y)+','+str(start_time_speed)+','+str(end_time_speed)
                    rospy.loginfo("Here: "+our_msg)
                    pub.publish(our_msg)
                    start_time_speed = None
                    end_time_speed= None
                    ratminx = None #reset things
                    finito = finito +1
                    #if(finito == 2):
                    #    sys.exit("Finito!")

            if viz:
                cv2.imshow("Security Feed", frame)
                cv2.imshow("Thresh", thresh)
                cv2.imshow("Frame Delta", frameDelta)
                #cv2.imshow('image',imm) #imgtcopy #img
                cv2.waitKey(1)

            frInd = frInd + 1
            start = timestamp
            #print "frInd is : %d" % frInd
            if divmod(frInd,100)[1] == 0: # if we go beyond 1m samples, close file and start a new one (prevents nastiness of huge files)
                logf.close()
                print "we had to start a new file"
                cur_datetime = datetime.datetime.now()
                cur_dtime_str = cur_datetime.strftime('%Y-%m-%d_%Hh%Mm%Ss')
                logfname = 'arenatracker_%s.csv' % cur_dtime_str #modified this to do in the current file
                logf = open(logfname, 'w', 0) # 0 forces flush after each write call (buffer = 0)
                logf.write('framenum,UTCtime,fps,animx,animy,bbx,bby,bbw,bbh,bbtheta\n')

        # close file
        # logf.close()
        # c.stop_capture()
        # c.disconnect()
        # if viz:
            # cv2.destroyAllWindows()
        # print '\nQuiting...'

    except KeyboardInterrupt:
        # close file
        logf.close()
        c.stop_capture()
        c.disconnect()
        if viz:
            cv2.destroyAllWindows()
        rospy.signal_shutdown("KeyboardInterrupt ROS Shutting Down ")
        print '\nQuiting...'





if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("had rospy exception")
        pass
