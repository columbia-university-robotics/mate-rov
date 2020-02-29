import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils
from imutils import perspective
from imutils import contours

# Open the camera
cap = cv2.VideoCapture(1)

count = 0
while True:

    count = count + 1
    # read the background
    ret, image = cap.read()

    # (h, w) = image.shape[:2]
    # center = (w / 2, h / 2)
    # M = cv2.getRotationMatrix2D(center, 180, 1)
    # image = cv2.warpAffine(image, M, (h, w))

    resized = imutils.resize(image, width=1000, height=1000)
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (1, 1), 0)
    thresh = cv2.threshold(blurred, 160, 200, cv2.THRESH_BINARY)[1]
    # thresh = cv2.bitwise_not(thresh)

    # edged = cv2.Canny(gray, 100, 600)  phone image
    # edged = cv2.Canny(gray, 5, 70)
    # thresh = edged
    # find contours in the thresholded image and initialize the
    # shape detector

    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    copy = resized.copy()
    cv2.drawContours(copy, cnts, -1, (0, 255, 0), 3)


    count = 0
    for c in cnts:
        # This is to ignore that small hair countour which is not big enough
        if cv2.contourArea(c) < 50*50 and cv2.contourArea(c) > 10*10:
            count = count + 1
        
    print(count)

    output = resized.copy()
    for c in cnts:
    
        # draw in blue the contours that were founded
        cv2.drawContours(output, cnts, -1, 255, 3)

        # find the biggest countour (c) by the area
        c = max(cnts, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)

        # draw the biggest contour (c) in green
        cv2.rectangle(output,(x,y),(x+w,y+h),(255,255,255),2)

        # ******************
    
    cv2.imshow('frame', output)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # ******************
    cv2.imwrite('./raw' + str(count) + '.png' ,output)
    cv2.imwrite('./processed' + str(count) + '.png' ,output)

    
# Release the camera and destroy all windows         
cap.release()
cv2.destroyAllWindows()
