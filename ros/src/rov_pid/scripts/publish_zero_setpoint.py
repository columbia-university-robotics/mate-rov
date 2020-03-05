#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

pub = rospy.Publisher('/setpoint', Float64, queue_size=10)
rospy.init_node('zero_setpoint_publisher')
r = rospy.Rate(30) 
while not rospy.is_shutdown():
   pub.publish(0)
   r.sleep()
