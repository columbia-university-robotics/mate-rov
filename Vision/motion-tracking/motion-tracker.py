#William Xie 10/04/19
#code adapted from: https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
#no imutils though
import numpy as np
import cv2
import time
import os
import tracker
import collections

def nothing(x):
    pass

#takes input from cv2 sliders to adjust hsv min/max values
#shows trackbars on different window, 'track'
def refresh_color(min, max, frame):
    hsv_hue_min = cv2.getTrackbarPos('h_min', 'track')
    hsv_saturation_min = cv2.getTrackbarPos('s_min', 'track')
    hsv_value_min = cv2.getTrackbarPos('v_min', 'track')

    hsv_hue_max = cv2.getTrackbarPos('h_max', 'track')
    hsv_saturation_max = cv2.getTrackbarPos('s_max', 'track')
    hsv_value_max = cv2.getTrackbarPos('v_max', 'track')

    upper[2] = hsv_value_max
    upper[1] = hsv_saturation_max
    upper[0] = hsv_hue_max

    lower[0] = hsv_hue_min
    lower[1] = hsv_saturation_min
    lower[2] = hsv_value_min

    hue_lower = lower[0]*2
    hue_upper = upper[0]*2
    sat_lower = lower[1]/2.55
    sat_upper = upper[1]/2.55
    val_lower = lower[2]/2.55
    val_upper = upper[2]/2.55

    hue = "Hue: {}-{} deg".format(hue_lower, hue_upper)
    sat = "Sat: {:0.2f}-{}".format(sat_lower, sat_upper)
    val = "Val: {:0.2f}-{}".format(val_lower, val_upper)
    print("-"*50)
    print(hue)
    print(sat)
    print(val)

"""     print "-"*50
    print f"Hue: {lower[0]*2:d}-{upper[0]*2:d} deg"
    print f"Sat: {lower[1]/2.55:.0f}-{upper[1]/2.55:.0f} %"
    printf"Val: {lower[2]/2.55:.0f}-{upper[2]/2.55:.0f} %" """

#takes raw frame and performs color filtering and noise reduction
def process(frame):
    #blur, convert to HSV registering
    blurred = cv2.GaussianBlur(frame, (11,11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    #construct mask for desired color value
    #series of dilations and erosions to remove blobs in mask
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    #kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    #kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    #erode followed by dilate to reduce noise
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  

    #by this point, should be able to detect filtered out blue bottle cap
    return mask

#finds contours and does stuff to them
def track_targets(mask, frame):
    #finding contours in mask, init (x,y) of ball
    cntrs = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE)
    if len(cntrs) == 2:
        cntrs = cntrs[0]
    elif len(cntrs) == 3:
        cntrs = cntrs[1]
    #cntrs = imutils.grab_contours(cntrs)
    center = None

    for contour in cntrs:
        cv2.drawContours(mask, [contour], 0, (255, 0, 255), 3)

    #continue if >=1 contours found
    #doin stuff to contours
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
            #draw circle, centroid on frame, update list of points
            cv2.circle(frame, (int(x), int(y)), int(radius),
            (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.circle(mask, (int(x), int(y)), int(radius),
            (0, 255, 255), 2)
            cv2.circle(mask, center, 5, (0, 0, 255), -1)


    pts.appendleft(center)
    #draw trail of centroid points
    for i in range(1, len(pts)):
        #if either of tracked points are None, ignore
        if pts[i-1] is None or pts[i] is None:
            continue
        
        #otherwise, compute thickness of line and draw connector
        thickness = int(np.sqrt(10
            / float(i + 1))*2.5)
        cv2.line(frame, pts[i-1], pts[i], (0, 0, 255), thickness)
        cv2.line(mask, pts[i-1], pts[i], (0, 0, 255), thickness)

#WEBCAM: Logitech C270 Parameters
CAM_fov = 60
CAM_w = 640 #initially 1280
CAM_h = 480 #initially 720

setting = {'hue_min':0, 'hue_max': 180, 'sat_min': 0, 'sat_max': 255, 'val_min': 0, 'val_max': 255}

#def lower/upper boundaries of the bottle cap, --> "blue"
#range in HSV values, intialize list of range of hue, saturation, value
#adjust HERE to filter for different colors
lower = np.array([setting['hue_min'], setting['sat_min'], setting['val_min']])
upper = np.array([setting['hue_max'], setting['sat_max'], setting['val_max']])

""" lower = (0, 0, 0)
upper = (0, 0, 255) """
pts = collections.deque(maxlen=10)

w_name = 'webcam'
cv2.namedWindow(w_name)
cv2.namedWindow('track')

cv2.createTrackbar('h_min', 'track', setting['hue_min'], 180, nothing)
cv2.createTrackbar('s_min', 'track', setting['sat_min'], 255, nothing)
cv2.createTrackbar('v_min', 'track', setting['val_min'], 255, nothing)

cv2.createTrackbar('h_max', 'track', setting['hue_max'], 180, nothing)
cv2.createTrackbar('s_max', 'track', setting['sat_max'], 255, nothing)
cv2.createTrackbar('v_max', 'track', setting['val_max'], 255, nothing)

#vs = VideoStream(src=0).start()
vs = cv2.VideoCapture(0)

#delay for camera init
#time.sleep(2.0)
if vs.isOpened(): #test if frame opens
    rval, frame = vs.read()
else:
    rval = False

while rval: 
    rval, frame = vs.read()
    
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame  = cv2.resize(frame, (CAM_w, CAM_h))
    mask = process(frame)
    mask  = cv2.resize(mask, (CAM_w, CAM_h))
    #shows unfiltered frame
    refresh_color(lower, upper, frame)
    cv2.imshow('track', frame)
    #processes frame for contour detection
    #performs contour detection as desired
    #draws contours on mask and frame
    track_targets(mask, frame)

    #show process, detected name
    cv2.imshow(w_name, mask)
    key_press = cv2.waitKey(20)
    if key_press == 27:
        break

cv2.destroyWindow(w_name)
vs.release()


