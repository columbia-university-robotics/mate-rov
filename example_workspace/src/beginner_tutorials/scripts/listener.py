#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry



def callback(msg):
    h           = msg.header 
    child_frame = msg.child_frame_id 
    p           = msg.pose 
    
    rospy.loginfo(rospy.get_caller_id() + "I heard %s as the child frame", child_frame )
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", msg.pose.pose.position.z )
    
    
    
def callback2(msg):
    h           = msg.header 
    child_frame = msg.child_frame_id 
    p           = msg.pose 
    
    rospy.loginfo(rospy.get_caller_id() + "I heard %s as the child frame", child_frame )
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", msg.pose.pose.position.z )
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("our_odom_topic", Odometry, callback)
    rospy.Subscriber("our_odom_topic", Odometry, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    
    
    
roscore
  topics
    ----> the last msg on a topic ---> please send to the function within any subscribers
    
  node1
    publisher
    
  node2
    subscribers
    publishers
    
    
    
