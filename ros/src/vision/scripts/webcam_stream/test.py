import cv2
import time
#see: https://stackoverflow.com/questions/604749/how-do-i-access-my-webcam-in-python

cv2.namedWindow("webcam")
vc = cv2.VideoCapture(0)
vc.set(3, 720)
vc.set(4, 480)
#vc.set(cv2.CAP_PROP_FPS, 10) doesn't work

if vc.isOpened(): #test if frame opens
    rval, frame = vc.read()
else:
    rval = False

frame_rate = 10
prev = 0

while rval:
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #color options
    time_elapsed = time.time() - prev
    if time_elapsed > 1./frame_rate: #hardcoded fps limiting
        prev = time.time()
        cv2.imshow("webcam", frame)
        rval, frame = vc.read()
        key = cv2.waitKey(20)
        if key == 27:
            break

cv2.destroyWindow("webcam")
vc.release()