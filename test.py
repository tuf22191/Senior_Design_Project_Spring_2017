import cv2
import numpy as np
import flycapture2 as fc2
import time
import datetime
import sys


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
fps = 0
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



print "Starting tracking"
print "...press control-c to stop."
try:
    while True:
        print dir(c), "is the dir"
        zz =c.retrieve_buffer(im)

        img = np.array(zz) # these are coming in as crazy bayered things--and in grey
        #cv2.imshow('yellow',img)
        imm = img
        print "OHO"
        print (dir(zz))
        print "get format is: ", zz.get_format()
        print "OHO"
        timestamp = time.time()
        img = cv2.cvtColor(img, cv2.COLOR_BAYER_RG2RGB)# REal is COLOR_BAYER_BG2BGR) # debayer (pattern verified--cv2 likes BGR order, if vizing with cv2, use this)
        imm = img
        # imgr = img[:,:,2].copy()
        # # img = cv2.dilate(img, dilate_kernel, iterations = 2) # close image instead of dilating (or opening)
        # cx, cy, bbx, bby, bbw, bbh, bbtheta, imgtcopy = getXY(imgr, colorMin, colorMax, threshVal, size_rank_of_target)
        # # log data
        fps = 1./(timestamp-start)
        #logf.write('%d,%.4f,%.4f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f\n' % (frInd, timestamp, fps, cx, cy, bbx, bby, bbw, bbh, bbtheta))
        # print "FPS: %.3f, x: %.2f, y: %.2f\r" % (fps,cx,cy)
        # if timestamp >= progress_t + progress_interval:
        #     m, s = divmod(timestamp - first_start, 60)
        #     h, m = divmod(m, 60)
        #     print "Since start: %d h, %d m, %d s, FPS: %.3f, x: %.2f,y: %.2f\r" % (h, m, s, fps,cx,cy)
        #     progress_t = timestamp
        #     # threshVal = np.percentile(imgr, 0.5) # works with r led panel and bright window light

        if viz:
            # cv2.circle(imgr, (cx,cy), 10, (0,0,255), -1)
            cv2.imshow('image',imm) #imgtcopy #img
            cv2.waitKey(1)

        frInd = frInd + 1
        start = timestamp

        if divmod(frInd,1000000)[1] == 0: # if we go beyond 1m samples, close file and start a new one (prevents nastiness of huge files)
            logf.close()

            cur_datetime = datetime.datetime.now()
            cur_dtime_str = cur_datetime.strftime('%Y-%m-%d_%Hh%Mm%Ss')
            logfname = '//arenatracker_%s.csv' % cur_dtime_str
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
