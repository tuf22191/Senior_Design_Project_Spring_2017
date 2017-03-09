import cv2
import numpy as np
from pprint import pprint

img1 = cv2.imread('1.jpg')
#print img1
# from http://www.answers.opencv.org/question/29648/attributeerror-module-object-has-no-attribute-videowriter_fourcc/
fourcc = cv2.cv.CV_FOURCC(*'MJPG')#  fourcc = cv2.videowriter_fourcc(*'MJPG')
#cv2.imshow('hg',img1)
#cv2.waitKey(0)

height , width , layers =  img1.shape


arrOfFields = dir(img1.shape)
# print arrOfFields

#pprint(arrOfFields, indent =2 )
# print 'img1.shape is: %s'% (img1.shape,)
# print 'layers is: '+ str(layers)
video = cv2.VideoWriter('video.avi',fourcc,20,(width,height))
pprint(dir(video), indent =2 )
print ' is video'
counter = 0
try:

    cap = cv2.VideoCapture('example_01.mp4')

    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret==True:
            video.write(frame)
            print counter
            #cv2.imshow('frame',frame)
            counter = counter+1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break


except KeyboardInterrupt:
    # close file
    cap.release()
    video.release()
    cv2.destroyAllWindows()

while False:
    print 'writing...'
    video.write(img1)
    # cv2.imshow('hg',img1)
    # cv2.waitKey(0)
    counter = counter+1
# video.write(img2)
# video.write(img3)
#
