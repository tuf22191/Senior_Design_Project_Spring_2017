

# test3 has motion detection however it records a lot of objects with motion, we might want to average the number of frames


#now the rgb values are working
#help(flycapture2) , dirs(flycapture2) or obj inside instead, vars(obj)


import cv2
import numpy as np
import flycapture2 as fc2
import time
import datetime
import sys
import imutils
import argparse

def getXY(img, colorMin, colorMax, threshVal, size_rank_of_target):
    # imgt = cv2.inRange(img, colorMin, colorMax) # restricts to a color range
    rv, imgt = cv2.threshold(imgr, threshVal, 255, cv2.THRESH_BINARY_INV) #threshold(src, threshCuttOffVal, hiValPostThresh, threshType)
    imgtcopy = imgt.copy()
    ctours, h = cv2.findContours(imgt, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if ctours: # check to see we have contours
        ctour_areas = [cv2.contourArea(c) for c in ctours]
        ctour_ind = np.argsort(ctour_areas)
        target_ctour = np.where(ctour_ind == size_rank_of_target)
        print ctours[ctour_ind[target_ctour[0][0]]]
        cnt = ctours[ctour_ind[target_ctour[0][0]]]
        bbxy, bbwh, bbtheta = cv2.minAreaRect(cnt)
        M = cv2.moments(cnt)
        if int(M['m00']) > 0: # in case we lose all blobs
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        else:
            cx = 0
            cy = 0
    else:
        cx = 0
        cy = 0
        bbxy = (0,0)
        bbwh = (0,0)
        bbtheta = 0
    return cx, cy, bbxy[0], bbxy[1], bbwh[0], bbwh[1], bbtheta, imgtcopy



print "hellodfd"
print "dfdfdfdfddfdfdf"
# set up various things
colorMax = np.array([10, 10, 10],np.uint8) # 3 color channels
colorMin = np.array([250, 250, 250],np.uint8) # must be >= all values in colorThreshMin
dilate_kernel = np.ones((5,5), np.uint8)
threshVal = 20 #CHANGE THIS TO ADJUST LIGHTING CUT-OFFS
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
        frame = imutils.resize(frame, width=500)
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


        if(had_counter):
            cv2.rectangle(frame, (x_min_value, y_min_value), (x_max_value, y_max_value), (0, 255, 0), 2)
            text = "Occupied"
            cv2.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        if viz:
            cv2.imshow("Security Feed", frame)
            cv2.imshow("Thresh", thresh)
            cv2.imshow("Frame Delta", frameDelta)
            #cv2.imshow('image',imm) #imgtcopy #img
            cv2.waitKey(1)

        frInd = frInd + 1
        start = timestamp
        print "frInd is : %d" % frInd
        if divmod(frInd,100)[1] == 0: # if we go beyond 1m samples, close file and start a new one (prevents nastiness of huge files)
            logf.close()
            print "we had to start a new file"
            cur_datetime = datetime.datetime.now()
            cur_dtime_str = cur_datetime.strftime('%Y-%m-%d_%Hh%Mm%Ss')
            logfname = 'arenatracker_%s.csv' % cur_dtime_str #modified this to do in the current file
            logf = open(logfname, 'w', 0) # 0 forces flush after each write call (buffer = 0)
            logf.write('framenum,UTCtime,fps,animx,animy,bbx,bby,bbw,bbh,bbtheta\n')

except KeyboardInterrupt:
    # close file
    logf.close()
    c.stop_capture()
    c.disconnect()
    if viz:
        cv2.destroyAllWindows()
    print '\nQuiting...'
