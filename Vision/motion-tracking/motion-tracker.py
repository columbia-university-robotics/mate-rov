#William Xie 10/04/19
#wrote this on the bus to the hospital, and the best
#thing i had on me was a blue aquafina bottle cap
#code used from: https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
#since sample code used to detect a ball, likely will 
#struggle on side profile views of cap

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

#will not use args -v, --video
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, 
    help="max buffer size")
args = vars(ap.parse_args())

#def lower/upper boundaries of the bottle cap, --> "blue"
#range in HSV values, intialize list of range of hue, saturation, value
#adjust HERE to filter for different colors
lower_blue = (110, 50, 50)
upper_blue = (130, 255, 255)
pts = deque(maxlen=args["buffer"])

cv2.namedWindow("webcam")

#if no video, refer to webcam-->this is the desired behavior
if not args.get("video", False):
    vs = VideoStream(src=0).start()
#if video specified, grab reference to file
#might hardcode this later to search for index 1 webcam
else:
    vs = cv2.VideoCapture(args["video"])

#delay for camera init
time.sleep(2.0)

frame_rate = 10
prev = 0

while True: #read while true
    #adjust color
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #color options
    time_elapsed = time.time() - prev
    if time_elapsed > 1./frame_rate: #hardcoded fps limiting
        prev = time.time()
        frame = vs.read()

        #take frame from webcam or video file
        frame = frame [1] if args.get("video", False) else frame

        if frame is None:
            break

        #resize frame, blur, convert to HSV registering
        frame = imutils.resize(frame, width=640)
        blurred = cv2.GaussianBlur(frame, (11,11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        #construct mask for "blue" color
        #series of dilations and erosions to remove blobs in mask
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #by this point, should be able to detect filtered out blue bottle cap
        
        #finding contours in mask, init (x,y) of ball
        cntrs = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cntrs = imutils.grab_contours(cntrs)
        center = None

        #continue if >=1 contours found
        if len(cntrs) > 0:
            #find largest contour in mask
            #computer minimum enclosing circle and centroid
            circle = max(cntrs, key=cv2.contourArea)
            ((x,y), radius) = cv2.minEnclosingCircle(circle)
            moments_arr = cv2.moments(circle)
            center = (int(moments_arr["m10"] / moments_arr["m00"]),
                int(moments_arr["m01"] / moments_arr["m00"]))

            #cap must meet certain min radius
            #adjust HERE for different sized balls
            if radius > 0.5:
                #draw circle, centroid on frame, upadte list of points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

        pts.appendleft(center)
        #draw trail of centroid points
        for i in range(1, len(pts)):
            #if either of tracked points are None, ignore
            if pts[i-1] is None or pts[i] is None:
                continue
            
            #otherwise, compute thickness of line and draw connector
            thickness = int(np.sqrt(args["buffer"] 
                / float(i + 1))*2.5)
            cv2.line(frame, pts[i-1], pts[i], (0, 0, 255), thickness)

        #show frame
        cv2.imshow("webcam", frame)
        key_press = cv2.waitKey(20)
        if key_press == 27:
            break


if not args.get("video", False):
    vs.stop()
else:
    vs.release()
cv2.destroyWindow("webcam")
