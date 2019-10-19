#William Xie 10/04/19
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
import os

def nothing(x):
    pass

#will not use args -v, --video
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, 
    help="max buffer size")
args = vars(ap.parse_args())

setting = {'hue_min':0, 'hue_max': 180, 'sat_min': 0, 'sat_max': 255, 'val_min': 0, 'val_max': 255}
setting_file = os.path.join(os.path.expanduser('~'), '.multithresh.json')
if os.path.exists(setting_file):
    with open(setting_file, 'r') as f:
        setting = json.load(f)

#def lower/upper boundaries of the bottle cap, --> "blue"
#range in HSV values, intialize list of range of hue, saturation, value
#adjust HERE to filter for different colors
lower = np.array([setting['hue_min'], setting['sat_min'], setting['val_min']])
upper = np.array([setting['hue_max'], setting['sat_max'], setting['val_max']])

""" lower = (0, 0, 0)
upper = (0, 0, 255) """
pts = deque(maxlen=args["buffer"])

w_name = 'webcam'
cv2.namedWindow(w_name)
cv2.namedWindow('track')

cv2.createTrackbar('h_min', 'track', setting['hue_min'], 180, nothing)
cv2.createTrackbar('s_min', 'track', setting['sat_min'], 255, nothing)
cv2.createTrackbar('v_min', 'track', setting['val_min'], 255, nothing)

cv2.createTrackbar('h_max', 'track', setting['hue_max'], 180, nothing)
cv2.createTrackbar('s_max', 'track', setting['sat_max'], 255, nothing)
cv2.createTrackbar('v_max', 'track', setting['val_max'], 255, nothing)


#if no video, refer to webcam-->this is the desired behavior
if not args.get("video", False):
    vs = VideoStream(src=0).start()
#if video specified, grab reference to file
#might hardcode this later to search for index 1 webcam
else:
    vs = cv2.VideoCapture(args["video"])

#delay for camera init
time.sleep(2.0)

def refresh_color(min, max, frame):
    hsv_hue_min = cv2.getTrackbarPos('h_min', 'track')
    hsv_saturation_min = cv2.getTrackbarPos('s_min', 'track')
    hsv_value_min = cv2.getTrackbarPos('v_min', 'track')

    hsv_hue_max = cv2.getTrackbarPos('h_max', 'track')
    hsv_saturation_max = cv2.getTrackbarPos('s_max', 'track')
    hsv_value_max = cv2.getTrackbarPos('v_max', 'track')

    cv2.imshow('track', frame)

    upper[2] = hsv_value_max
    upper[1] = hsv_saturation_max
    upper[0] = hsv_hue_max

    lower[0] = hsv_hue_min
    lower[1] = hsv_saturation_min
    lower[2] = hsv_value_min

    print("-"*50)
    print(f"Hue: {lower[0]*2:d}-{upper[0]*2:d} deg")
    print(f"Sat: {lower[1]/2.55:.0f}-{upper[1]/2.55:.0f} %")
    print(f"Val: {lower[2]/2.55:.0f}-{upper[2]/2.55:.0f} %")

while True: 
    frame = vs.read()
    #take frame from webcam or video file
    frame = frame [1] if args.get("video", False) else frame

    if frame is None:
        break

    refresh_color(lower, upper, frame)

    #resize frame, blur, convert to HSV registering
    #frame = imutils.resize(frame, width=640)
    blurred = cv2.GaussianBlur(frame, (11,11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    #construct mask for desired color value
    #series of dilations and erosions to remove blobs in mask
    mask = cv2.inRange(hsv, lower, upper)
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
    cv2.imshow(w_name, mask)
    key_press = cv2.waitKey(20)
    if key_press == 27:
        break


if not args.get("video", False):
    vs.stop()
else:
    vs.release()
cv2.destroyWindow(w_name)
