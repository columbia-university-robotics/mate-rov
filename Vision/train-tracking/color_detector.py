#!/usr/bin/env python

'''hand_detector is a program which detects, in sequence, a closed fist in the center of an image
and a five! (splayed hand) in any of the four corners of the succeeding image
for COMS 4735: Visual Interfaces for Computers
Parts of code adapated from another vision-related project, seen here:
https://github.com/columbia-university-robotics/mate-rov/blob/Vision/Vision/motion-tracking/motion-tracker.py
'''

import time
import os
import collections

import numpy as np
import cv2

__author__ = "William Xie, UNI: wx2214"

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
    # print("-"*50)
    # print(hue)
    # print(sat)
    # print(val)

#compares current frame to frame captured on startup
#if significant difference over threshold, set to white 
#I tried to hack this together and it DID NOT WORK
def strip(frame, reference):
    diff = cv2.absdiff(frame, reference)
    diff_mask = diff

    threshold = [180, 0, 255]
    th_mask = diff_mask > threshold
    #print(th_mask[[[1,1,1]]])
    #initialize background subtraction frame with zeros
    #bg_mask = np.zeros_like(diff_mask, np.uint8)
    bg_mask = frame
    # height, width, depth = bg_mask.shape
    # for i in range(0, height):
    #     for j in range(0, width):
            
    bg_mask[th_mask] = diff_mask[th_mask]

    return bg_mask

#takes raw frame and performs color filtering and noise reduction
def process(frame):
    #blur, convert to HSV registering
    mask = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.GaussianBlur(mask, (11,11), 0)

    #construct mask for desired color value
    #series of dilations and erosions to remove blobs in mask
    mask = cv2.inRange(mask, lower, upper)
    #erode followed by dilate to reduce noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    #by this point, should be able to detect filtered out blue bottle cap
    return mask

def draw_lines(p_frame, p_mask, pts, color):
    #draw trail of centroid points
    #code adapted from: https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
    for i in range(1, len(pts)):
        #if either of tracked points are None, ignore
        if pts[i-1] is None or pts[i] is None:
            continue
        
        #otherwise, compute thickness of line and draw connector
        thickness = int(np.sqrt(10
            / float(i + 1))*2.5)
        cv2.line(p_frame, pts[i-1], pts[i], color, thickness)
        cv2.line(p_mask, pts[i-1], pts[i], color, thickness)

def print_text(p_mask, p_frame, text, location, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = color
    line_type = 2

    cv2.putText(p_mask, text, location, font,
        font_scale, font_color, line_type)
    cv2.putText(p_frame, text, location, font,
        font_scale, font_color, line_type)

#finds and tracks contours, center of mass of filtered image
def track_centroid(p_mask, p_frame):
    #establish color for when drawn
    color = (180,255,255)
    location = (5, 450)

    center_mask = None

    M = cv2.moments(p_mask)
    center_mask = (int(M["m10"] / M["m00"]),
        int(M["m01"] / M["m00"]))
    center_x = center_mask[0]
    center_y = center_mask[1]
    com = f"COM: ({center_x}, {center_y})" #center of mass
    print_text(p_mask, p_frame, com, location, color)

    pts.appendleft(center_mask)
    
    draw_lines(p_frame, p_mask, pts, color)

def draw_rectangles(p_mask, p_frame):
    color = (180,255,255)
    location = (5, 450)

    cntrs = cv2.findContours(p_mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE)
    if len(cntrs) == 2:
        cntrs = cntrs[0]
    elif len(cntrs) == 3:
        cntrs = cntrs[1]

    for contour in cntrs:
        cv2.drawContours(p_mask, [contour], 0, (255, 0, 255), 3)

    if len(cntrs) > 0:
        for contour in cntrs:
            #simple bounding rectangle
            # x, y, r_width, r_height = cv2.boundingRect(contour)
            # rect_frame = cv2.rectangle(p_frame, (x, y), (x+r_width, y+r_height), color, 2)
            # rect_mask = cv2.rectangle(p_mask, (x, y), (x+r_width, y+r_height), color, 2)
            
            #rotated rectangle
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(p_frame, [box], 0, color, 2)
            cv2.drawContours(p_mask, [box], 0, color, 2)


#checks that fist is shown and
#that it meets location requirements
def check_fist(p_mask, p_frame):
    check_fist = True
    color = (120,255,255)

    #finding contours in mask, init (x,y) of ball
    f_contours = cv2.findContours(p_mask, cv2.RETR_TREE,
        cv2.CHAIN_APPROX_NONE)

    #varying OpenCV versions will return varying lengths of contour tuples
    if len(f_contours) == 2:
        contours = f_contours[0]
    elif len(f_contours) == 3:
        contours = f_contours[1]

    center = None
    max_contour = max(contours, key=cv2.contourArea)

    #approximates polygon from contour
    #draws convex hull over approximated polygon    
    epsilon = 0.03 * cv2.arcLength(max_contour, True)
    approx_fist = cv2.approxPolyDP(max_contour, epsilon, True)
    
    hull = cv2.convexHull(approx_fist)
    # cv2.drawContours(p_frame, [hull], 0, color, 3)
    # cv2.drawContours(p_mask, [hull], 0, color, 3)

    x, y, r_width, r_height = cv2.boundingRect(hull)
    rect = cv2.rectangle(p_frame,(x,y),(x+r_width,y+r_height),color,2)

    if len(contours) > 0:
        fist_area = cv2.contourArea(hull)
        moments_arr = cv2.moments(hull)
        center = (int(moments_arr["m10"] / moments_arr["m00"]),
            int(moments_arr["m01"] / moments_arr["m00"]))
        # print(center)
        # print(fist_area)
        center_x = center[0]
        center_y = center[1]
        if fist_area >= 10000 and fist_area <= 80000 and check_fist_center(center, r_width, r_height):
            msg = f"check_fist: {check_fist}"
            print_text(p_mask, p_frame, msg, (200,240), (180,255,255))  
        else:
            check_fist = False
            msg = f"check_fist: {check_fist}"
            print_text(p_mask, p_frame, msg, (200,240), (180,255,255))  
        com = f"MAX CNTR: ({center_x}, {center_y})" #center of mass
        location = (280, 450)
        print_text(p_mask, p_frame, com, location, color)
            
    pts_fist.appendleft(center)
    draw_lines(p_frame, p_mask, pts_fist, color)

    return check_fist

#checks that fist is in the central 320x240 rectangle
def check_fist_center(center, r_width, r_height):
    fist_center = False
    #first check that fist is "proportionally" correct
    r_ratio = r_width/r_height
    #print(r_ratio)
    if (r_ratio >=2.35 or r_ratio <= 1.05):
        return False

    #camera params
    width = 640
    height = 480
    m_x = width/2 
    m_y = height/2
    x_off = width*0.25
    y_off = height*0.25

    x = center[0]
    y = center[1]

    if (x >= m_x - x_off and x <= m_x + x_off  
        and y >= m_y - y_off and y <= m_y + y_off):
        fist_center = True
    else:
        fist_center = False

    #print(fist_center)
    return fist_center

#checks that splayed hand (five!) is shown and
#that it meets location requirements
def check_five(p_mask, p_frame):
    check_five = True
    #establish color for when drawn
    color = (255,255,255)

    #finding contours in mask, init (x,y) of ball
    f_contours = cv2.findContours(p_mask, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE)

    #varying OpenCV versions will return varying lengths of contour tuples
    if len(f_contours) == 2:
        contours = f_contours[0]
    elif len(f_contours) == 3:
        contours = f_contours[1]

    center = None
    max_contour = max(contours, key=cv2.contourArea)

    #approximates polygon from contour
    #epsilon constant adjusted from 0.1 to 0.03, results very good    
    epsilon = 0.05 * cv2.arcLength(max_contour, True)
    approx_five = cv2.approxPolyDP(max_contour, epsilon, True)
    # cv2.drawContours(p_frame, [approx_fist], -1, color, 3)
    # cv2.drawContours(p_mask, [approx_fist], -1, color, 3)

    #draws convex hull over approximated polygon
    hull = cv2.convexHull(max_contour)
    cv2.drawContours(p_frame, [hull], 0, color, 10)
    cv2.drawContours(p_mask, [hull], 0, color, 10)  

    x, y, r_width, r_height = cv2.boundingRect(hull)
    rect = cv2.rectangle(p_frame,(x,y),(x+r_width,y+r_height),color,2)

    #continue if >=1 contours found
    if len(contours) > 0:
        #calculate solidity of piece
        hull_area = cv2.contourArea(max_contour)
        rect_area = r_width*r_height
        five_solidity = float(hull_area)/rect_area
        #print(five_solidity)

        hull_defects = cv2.convexHull(max_contour, returnPoints=False)
        defects = cv2.convexityDefects(max_contour, hull_defects)
        qty_defects = len(defects)
             
        #compute equivalent area circle and centroid
        circle = hull
        #((center_x,center_y), radius) = cv2.minEnclosingCircle(circle)
        circle_area = hull_area
        equiv_diameter = np.sqrt(4*circle_area/np.pi)
        radius = equiv_diameter/2
        moments_arr = cv2.moments(circle)
        center = (int(moments_arr["m10"] / moments_arr["m00"]),
            int(moments_arr["m01"] / moments_arr["m00"]))
        center_x = center[0]
        center_y = center[1]

        #cap must meet certain min radius
        #check that at least 70% of hand is in one of the 160x120 quadrants
        if (radius >= 40 and radius <= 220 and 
            qty_defects <= 20 and
            five_solidity <= 0.60 and
            check_five_corner(center, radius, r_width, r_height)):
            #draw circle, centroid on frame, update list of points
            cv2.circle(p_frame, (int(center_x), int(center_y)), int(radius),
            (255, 255, 255), 2)
            cv2.circle(p_frame, center, 5, (255, 255, 255), -1)
            cv2.circle(p_mask, (int(center_x), int(center_y)), int(radius),
            (0, 255, 255), 2)
            cv2.circle(p_mask, center, 5, (0, 0, 255), -1)
            msg = f"check_five: {check_five}"
            print_text(p_mask, p_frame, msg, (200,240), (180,255,255))          

        else:
            check_five = False
            msg = f"check_five: {check_five}"
            print_text(p_mask, p_frame, msg, (200,240), (180,255,255))  

        #put (x,y) of circle's radius to text on cv2 screen

        com = f"MAX CNTR: ({center_x}, {center_y})" #center of mass
        location = (280, 450)
        print_text(p_mask, p_frame, com, location, color)

    pts_five.appendleft(center)
    draw_lines(p_frame, p_mask, pts_five, color)

    return check_five

#checks that splayed hand occupies at least 70% of one 160x120 quadrant
#visually, there is a deadzone in the shape of a cross running along the 
#midlines of the screen, and if the center of the circle falls in that
#deadzone, then check_five_corner returns false
def check_five_corner(center, radius, r_width, r_height):
    five_corner = False

    r_ratio = r_width/r_height
    #print(r_ratio)
    if (r_ratio >=1.2):
        return False

    sector_ratio = 0.2663675 #magic constant, see report for details
    #camera params
    width = 640
    height = 480
    m_x = width/2 
    m_y = height/2
    x_off = radius*sector_ratio
    y_off = radius*sector_ratio

    x = center[0]
    y = center[1]

    #top-left quadrant (top left is (0,0))
    if (x >= 0 and x <= m_x - x_off  
        and y >= 0 and y <= m_y - y_off):
        five_corner = True
    #top-right quadrant
    elif (x >= m_x + x_off and x <= width  
        and y >= 0 and y <= m_y - y_off):
        five_corner = True
    #bottom-left quadrant
    elif (x >= 0 and x <= m_x - x_off  
        and y >= m_y + y_off and y <= height):
        five_corner = True       
    elif (x >= m_x + x_off and x <= width  
        and y >= m_y + y_off and y <= height):
        five_corner = True
    else:
        five_corner = False

    #print(five_corner)
    return five_corner

#initiates grammar sequence for visual lock/unlock
#and updates control state
def state_controller(mask, fg_mask, dict):
    print(dict["TEMPORARY_CYCLES"])
    #determine state and PERSISTENT_CYCLES and output to image
    state_name = dict["CURRENT_STATE"]
    state = f"STATE: {state_name}"
    cycle_count = dict["PERSISTENT_CYCLES"]
    cycle = f"CYCLE: {cycle_count}"

    location = (10, 25)
    color = (255,255,255)
    print_text(mask, fg_mask, state, location, color)
    print_text(mask, fg_mask, cycle, (460, 25), color)
    
    #determine next state from BUFFER position
    if dict["CURRENT_STATE"] is "BUFFER" and dict["BUFFER"] is True:
        #going from CHECKING_FIST to CHECKING_FIVE            
        if dict["CHECKING_FIST"] is True:
            if dict["TEMPORARY_CYCLES"] == 0:
                dict["TEMPORARY_CYCLES"] = dict["PERSISTENT_CYCLES"]
            elif dict["TEMPORARY_CYCLES"] > 0:
                time_elapsed = dict["PERSISTENT_CYCLES"] - dict["TEMPORARY_CYCLES"]
                print_text(mask, fg_mask, "check_fist() PASS, check_five()", (100, 320), color)
                if time_elapsed == 20:
                    dict["BUFFER"] = False
                    dict["CHECKING_FIST"] = False
                    dict["CHECKING_FIVE"] = True
                    dict["CURRENT_STATE"] = "CHECKING_FIVE"
                    dict["TEMPORARY_CYCLES"] = 0
        elif dict["UNLOCKED"] is True: 
            if dict["TEMPORARY_CYCLES"] == 0:
                dict["TEMPORARY_CYCLES"] = dict["PERSISTENT_CYCLES"]
            elif dict["TEMPORARY_CYCLES"] > 0:
                time_elapsed = dict["PERSISTENT_CYCLES"] - dict["TEMPORARY_CYCLES"]
                print_text(mask, fg_mask, "return to check_fist()", (100, 320), color)
                if time_elapsed == 20:
                    dict["BUFFER"] = False
                    dict["UNLOCKED"] = False
                    dict["CHECKING_FIST"] = True
                    dict["CURRENT_STATE"] = "CHECKING_FIST"
                    dict["TEMPORARY_CYCLES"] = 0
        elif dict["CHECKING_FIVE"] is False:
            if dict["TEMPORARY_CYCLES"] == 0:
                dict["TEMPORARY_CYCLES"] = dict["PERSISTENT_CYCLES"]
            elif dict["TEMPORARY_CYCLES"] > 0:
                time_elapsed = dict["PERSISTENT_CYCLES"] - dict["TEMPORARY_CYCLES"]
                print_text(mask, fg_mask, "check_five() FAIL, check_fist()", (100, 320), color)
                if time_elapsed == 20:
                    dict["BUFFER"] = False
                    dict["CHECKING_FIST"] = True
                    dict["CHECKING_FIVE"] = False
                    dict["CURRENT_STATE"] = "CHECKING_FIST"
                    dict["TEMPORARY_CYCLES"] = 0
                #going from UNLOCKED back to CHECKIN_FIST

    #determine next state from UNLOCKED position
    #UNLOCKED only occurs if CHECKING_FIVE passes
    #and is active for only 2 seconds
    if dict["CURRENT_STATE"] is "UNLOCKED":
        if dict["TEMPORARY_CYCLES"] == 0:
            dict["TEMPORARY_CYCLES"] = dict["PERSISTENT_CYCLES"]   
        elif dict["TEMPORARY_CYCLES"] > 0:
            time_elapsed = dict["PERSISTENT_CYCLES"] - dict["TEMPORARY_CYCLES"]
            print_text(mask, fg_mask, "check_five(), check_fist() PASS", (100, 320), color)
            if time_elapsed == 20:
                dict["BUFFER"] = True
                dict["CHECKING_FIST"] = False
                dict["CHECKING_FIVE"] = False
                dict["CURRENT_STATE"] = "BUFFER"
                dict["TEMPORARY_CYCLES"] = 0        

    #state logic for when state is CHECKING_FIST
    #I should rewrite all this to use CURRENT_STATE
    if dict["CURRENT_STATE"] is "CHECKING_FIST":
        dict["CYCLES_FIST"] += 1
        if check_fist(mask, fg_mask):
            dict["COUNT_FIST"] += 1

        dict["AVG_FIST"] = float(dict["COUNT_FIST"])/dict["CYCLES_FIST"]

        if dict["CYCLES_FIST"] == 20:
            print(dict["AVG_FIST"])

            if dict["AVG_FIST"] >= 0.75:
                dict["BUFFER"] = True
                dict["CURRENT_STATE"] = "BUFFER"
                dict["CYCLES_FIST"] = 0
                dict["COUNT_FIST"] = 0
            else:
                dict["CYCLES_FIST"] = 0
                dict["COUNT_FIST"] = 0

    #state logic for when state is CHECKING_FIST
    #a little trickier than fist since it can either unlock or go back to fist
    if dict["CURRENT_STATE"] is "CHECKING_FIVE":
        dict["CYCLES_FIVE"] += 1
        if check_five(mask, fg_mask):
            dict["COUNT_FIVE"] += 1

        dict["AVG_FIVE"] = float(dict["COUNT_FIVE"])/dict["CYCLES_FIVE"]
        if dict["CYCLES_FIVE"] == 20:
            if dict["AVG_FIVE"] >= 0.75:
                dict["UNLOCKED"] = True
                dict["CURRENT_STATE"] = "UNLOCKED"
                dict["CYCLES_FIVE"] = 0
                dict["COUNT_FIVE"] = 0
            else:
                dict["BUFFER"] = True
                dict["CURRENT_STATE"] = "BUFFER"
                dict["CYCLES_FIVE"] = 0
                dict["COUNT_FIVE"] = 0
                dict["CHECKING_FIVE"] = False
            

    return None

def begin_control(mask, fg_mask, dict):
    dict["PERSISTENT_CYCLES"] += 1

#WEBCAM: Logitech C270 Parameters
CAM_fov = 60
CAM_w = 640 #initially 1280
CAM_h = 480 #initially 720

#empirically determined hsv skin color range:
#hue: 0-50
#sat: .25-.65 (64-166)
#val: 100-
setting = {'hue_min':0, 'hue_max': 165, 'sat_min': 200, 'sat_max': 255, 'val_min': 145, 'val_max': 235}
lower = np.array([setting['hue_min'], setting['sat_min'], setting['val_min']])
upper = np.array([setting['hue_max'], setting['sat_max'], setting['val_max']])

w_name = 'webcam'
cv2.namedWindow(w_name)
cv2.namedWindow('track')
cv2.namedWindow('hsv')

#live-adjust HSV threshold values
cv2.createTrackbar('h_min', 'track', setting['hue_min'], 180, nothing)
cv2.createTrackbar('s_min', 'track', setting['sat_min'], 255, nothing)
cv2.createTrackbar('v_min', 'track', setting['val_min'], 255, nothing)

cv2.createTrackbar('h_max', 'track', setting['hue_max'], 180, nothing)
cv2.createTrackbar('s_max', 'track', setting['sat_max'], 255, nothing)
cv2.createTrackbar('v_max', 'track', setting['val_max'], 255, nothing)

if __name__ == "__main__":
    vs = cv2.VideoCapture(0)
    
    if vs.isOpened(): #test if frame opens
        rval, frame = vs.read()
    else:
        rval = False
    #background subtraction before image processing
    
    #this worked too well
    #back_sub = cv2.createBackgroundSubtractorMOG2()
    #reference = frame

    #instantiate lists which hold centroids 
    pts = collections.deque(maxlen=10)
    pts_five = collections.deque(maxlen=10)
    pts_fist = collections.deque(maxlen=10)

    frame_rate = 10
    prev = 0

    
    #states


    #state values
    TEMPORARY_CYCLES  = 0    #one cycle every 0.1 seconds, for user to stash times in
    PERSISTENT_CYCLES = 0    #one cycle every 0.1 seconds 
    CYCLES_FIST       = 0    #one cycle every 0.1 seconds, resets with each new state
    CYCLES_FIVE       = 0    #one cycle every 0.1 seconds, resets with each new state
    COUNT_FIST        = 0
    COUNT_FIVE        = 0
    AVG_FIST          = None
    AVG_FIVE          = None

    state_dict = {"CURRENT_STATE": None, 
                "CHECKING_FIST": None, 
                "CHECKING_FIVE": None, 
                "UNLOCKED": False,
                "BUFFER": False,
                "TEMPORARY_CYCLES": 0, 
                "PERSISTENT_CYCLES": 0, 
                "CYCLES_FIST": 0,
                "CYCLES_FIVE": 0, 
                "COUNT_FIST": 0, 
                "COUNT_FIVE": 0, 
                "AVG_FIST": 0, 
                "AVG_FIVE": 0}

    state_dict["CURRENT_STATE"] = "CHECKING_FIST"
    state_dict["CHECKING_FIST"] = True
    state_dict["UNLOCKED"] = False
    state_dict["Buffer"] = False

    while rval: 
        time_elapsed = time.time() - prev
        rval, frame = vs.read()

        #frame rate hack to reduce bandwidth load, running 4 screens
        #all with significant image processing, not very resource-friendly
        if time_elapsed > 1./frame_rate:
            prev = time.time()

            #shows unfiltered frame
            refresh_color(lower, upper, frame)
            cv2.resizeWindow('track', 640,480)
            cv2.imshow('track', frame)

            fg_mask = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = process(fg_mask) #equivalently, the "isSkin() method"
            
            track_centroid(mask, fg_mask)
            draw_rectangles(mask, fg_mask)
            #not needed for 2/22 pool test
            #begin_control(mask, fg_mask, state_dict)
            #state_controller(mask, fg_mask, state_dict)            

            
            #show process, detected name
            cv2.resizeWindow(w_name, 640,480)
            cv2.imshow(w_name, mask)
            cv2.resizeWindow('hsv', 640,480)
            cv2.imshow('hsv', fg_mask)

        key_press = cv2.waitKey(27)
        if key_press == 27: #escape key
            break

    cv2.destroyWindow(w_name)
    vs.release()