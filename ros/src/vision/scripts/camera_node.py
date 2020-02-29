#!/usr/bin/env python
# 
# simple camera node. Publishes frames
# from a live camera. 
# 
# Written by Neil Nie for CURC
# (c) Neil Nie, 2019
#

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import cv2


def openCamera(cam_indx):

    # create node and publisher
    rospy.init_node('camera_node')
    image_publisher = rospy.Publisher('sensor/camera/image', Image, queue_size=5)
    rate = rospy.Rate(30)

    # create cv bridge
    bridge = CvBridge()

    # declare cv2 cam
    cam = cv2.VideoCapture(cam_index)

    while not rospy.is_shutdown():

        ret_val, img = cam.read()
        image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")        
        
        try:
            image_publisher.publish(image_message)
        except CvBridgeError as e:
            print(e)

        if (cv2.waitKey(1) == 27)
            break

	rate.sleep()


if __name__ == "__main__"

    try:
        openCamera()
    except rospy.ROSInterruptException:
        pass

