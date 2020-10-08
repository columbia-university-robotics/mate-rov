#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry

def talker():

    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('our_odom_topic', Odometry, queue_size=10)
    
    odom_msg = Odometry()
    odom_msg.pose.pose.position.z = 1
    
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        odom_msg.pose.pose.position.z += 1
        #rospy.loginfo(odom_msg)
        pub.publish(odom_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
