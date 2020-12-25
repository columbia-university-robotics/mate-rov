#!/usr/bin/env python
# This script is a modification from the ROS teleop_keyboard package
# https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
# modifications by : Jonathan Sanabria
from __future__ import print_function

import threading

import roslib
import rospy

#from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float32

import sys, select, termios, tty
import numpy as np

msg = """
Reading from the keyboard  and Publishing to Float64MultiArray!
---------------------------
Moving around:
        w         r               i         
   a    s    d    f           j   k   l
   z         c


w/s : Z-axis  up/down 
a/d : yaw   left/right
z/c : roll   ccw/cw
r/f : pitch   up/down
i/k :    forward/reverse
j/l : truck left/right 

anything else : stop

CTRL-C to quit the keyboard control and then one more time to close the simulation

"""

ZERO_ARRAY = np.array([ 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0])
moveBindings={
        'w':np.array([ 0.0, 0.0, 0.0, 0.0,  1.0, 1.0,-1.0,-1.0]), # up
        's':np.array([ 0.0, 0.0, 0.0, 0.0, -1.0,-1.0, 1.0, 1.0]), # down
        'a':np.array([ 0.0, 0.0, 0.0, 0.0, -1.0,-1.0,-1.0,-1.0]), # yaw left  ( + theta )
        'd':np.array([ 0.0, 0.0, 0.0, 0.0,  1.0, 1.0, 1.0, 1.0]), # yaw right ( - theta )
        'i':np.array([ 1.0, 1.0, 1.0, 1.0,  0.0, 0.0, 0.0, 0.0]), # forward
        'k':np.array([-1.0,-1.0,-1.0,-1.0,  0.0, 0.0, 0.0, 0.0]), # reverse
        'j':np.array([ 1.0,-1.0, 1.0,-1.0,  0.0, 0.0, 0.0, 0.0]), # truck left  
        'l':np.array([-1.0, 1.0,-1.0, 1.0,  0.0, 0.0, 0.0, 0.0]), # truck right 
        'z':np.array([ 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,-1.0, 1.0]), # roll counter-clockwise 
        'c':np.array([ 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 1.0,-1.0]), # roll clockwise 
        'r':np.array([ 0.0, 0.0, 0.0, 0.0,  1.0,-1.0, 0.0, 0.0]), # pitch up
        'f':np.array([ 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0])  # pitch down 
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/rov/joint_motor_controller/command', Float64MultiArray, queue_size=4)
        self.propellor_array = ZERO_ARRAY
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, prop_value_ar ):
        self.condition.acquire()
        # IDEALLY WE"D HAVE THE ODOMTERY MATHSIM EQUATIONS HERE TO CALCULATE THE MOTOR VALUES FROM THE CMD_vel (directions and such) THAT IS DESIRED
        self.propellor_array += prop_value_ar 

        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(ZERO_ARRAY)
        self.join()

    def run(self):
        
        #twist = Twist()
        # motor value array 
        f = Float64MultiArray() 
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into ros multi array message.
            f.data = list(self.propellor_array)

            self.condition.release()

            # Publish.
            self.publisher.publish(f)   # main publisher  

        # Publish stop message when thread exits.
        self.propellor_array = ZERO_ARRAY
        f.data = list(self.propellor_array)
        self.publisher.publish(f)   # main publisher


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)
    """
    x = 0 # main publisher
    y = 0 # main publisher
    z = 0 # main publisher
    th = 0 # main publisher
    status = 0 # main publisher
    """
    update_prop_values_by = ZERO_ARRAY
    fr_ = 0   # update_prop_values_by[0] 
    fl_ = 0   # update_prop_values_by[1]
    bl_ = 0   # update_prop_values_by[2]
    br_ = 0   # update_prop_values_by[3]
    fm_ = 0   # update_prop_values_by[4]
    bm_ = 0   # update_prop_values_by[5]
    rm_ = 0   # update_prop_values_by[6]  
    lm_ = 0   # update_prop_values_by[7]

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(update_prop_values_by) # main publisher

        print(msg)
        #print(vels(speed,turn))
        while not rospy.is_shutdown():
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                update_prop_values_by = moveBindings[key]
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and np.all(update_prop_values_by==0):
                    continue
                update_prop_values_by = ZERO_ARRAY
                if  key == '\x03':
                    break
 
            pub_thread.update(update_prop_values_by) # main publisher

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

