#!/usr/bin/python
# -*- coding: utf-8 -*-

import pygame
import rospy
from std_msgs.msg import Float32


class Controller(object):

    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self.actions = {}
        self.lastActions = {}
        self.joystick_count = pygame.joystick.get_count()

        # http://wiki.ros.org/msg   #    Float32

	#publishing for the rov:
        rospy.init_node('controller')
        rov_left_vert_pub = rospy.Publisher('controller/rov/left_vert',
                Float32, queue_size=10)
        rov_left_hori_pub = rospy.Publisher('controller/rov/left_hori',
                Float32, queue_size=10)
        rov_right_vert_pub = rospy.Publisher('controller/rov/right_vert',
                Float32, queue_size=10)
        rov_right_hori_pub = rospy.Publisher('controller/rov/right_hori',
                Float32, queue_size=10)
        rov_right_topbumper_pub = \
            rospy.Publisher('controller/rov/right_topbumper', Float32,
                            queue_size=10)
        rov_right_botbumper_pub = \
            rospy.Publisher('controller/rov/right_bottombumper', Float32,
                            queue_size=10)

	#publishing for the claw:
        claw_left_vert_pub = rospy.Publisher('controller/claw/left_vert',
                Float32, queue_size=10)
        claw_left_hori_pub = rospy.Publisher('controller/claw/left_hori',
                Float32, queue_size=10)
        claw_right_vert_pub = rospy.Publisher('controller/claw/right_vert',
                Float32, queue_size=10)
        claw_right_hori_pub = rospy.Publisher('controller/claw/right_hori',
                Float32, queue_size=10)
        claw_right_topbumper_pub = \
            rospy.Publisher('controller/claw/right_topbumper', Float32,
                            queue_size=10)
        claw_right_botbumper_pub = \
            rospy.Publisher('controller/claw/right_bottombumper', Float32,
                            queue_size=10)
        r = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.lastActions = self.actions
            action_dict = self.get_actions()	#controller_0 is rov; controller_1 is claw
            for (k, v) in action_dict.items():
                if 'joystick' in k:

                                    # publish necessary joystick values

                    # print k
                    if 'left' in k:
                        print 'IN LEFT '
                        if 'vertical' in k:  # and self.hasChanged("JLV") ):

                                            # print( "IN JLV ")
			    if 'controller_0' in k:
                                rov_left_vert_pub.publish(v)
			    elif 'controller_1' in k:
				claw_left_vert_pub.publish(v)

                        elif 'horizontal' in k:

                                             # and self.hasChanged("JLH") ):
			    if 'controller_0' in k:
                                rov_left_hori_pub.publish(v)
			    elif 'controller_1' in k:
                                claw_left_hori_pub.publish(v)
                    elif 'right' in k:
                        if 'vertical' in k:  # and self.hasChanged("JRV") ):

                                            # print( "IN JRV ")
			    if 'controller_0' in k:
                                rov_right_vert_pub.publish(v)
			    elif 'controller_1' in k:
                                claw_right_vert_pub.publish(v)
                        elif 'horizontal' in k:

                                             # and self.hasChanged("JRH") ):
			    if 'controller_0' in k:
                                rov_right_hori_pub.publish(v)
			    elif 'controller_1' in k:
                                claw_right_hori_pub.publish(v)
                if 'button' in k:

                                # publish necessary button values

                    if 5 in k:  # and self.hasChanged("B5") ):

                                    # print( "IN B5 ")
			if 'controller_0' in k:
                            rov_right_topbumper_pub.publish(v)
			elif 'controller_1' in k:
                            claw_right_topbumper_pub.publish(v)
                    elif 7 in k:

                              # and self.hasChanged("B7") ):
			if 'controller_0' in k:
                            rov_right_botbumper_pub.publish(v)
			elif 'controller_1' in k:
                            claw_right_botbumper_pub.publish(v)
                if 'arrow' in k:

                                    # publish necessary arrow values

                    pass

            r.sleep()

    def hasChanged(self, abbreviated_key):
        if abbreviated_key == 'JLV':
            if self.actions[('joystick', 'left', 'vertical')] \
                == self.lastActions[('joystick', 'left', 'vertical')]:
                return False
            return True
        elif abbreviated_key == 'JLH':
            if self.actions[('joystick', 'left', 'horizontal')] \
                == self.lastActions[('joystick', 'left', 'horizontal')]:
                return False
            return True
        elif abbreviated_key == 'JRV':
            if self.actions[('joystick', 'right', 'vertical')] \
                == self.lastActions[('joystick', 'right', 'vertical')]:
                return False
            return True
        elif abbreviated_key == 'JRH':
            if self.actions[('joystick', 'right', 'horizontal')] \
                == self.lastActions[('joystick', 'right', 'horizontal'
                                    )]:
                return False
            return True
        elif abbreviated_key == 'B5':

                                         # right top bumper

            if self.actions[('button', 5)] == self.lastActions[('button'
                    , 5)]:
                return False
            return True
        elif abbreviated_key == 'B7':

                                         # right bottom bumper

            if self.actions[('button', 7)] == self.lastActions[('button'
                    , 7)]:
                return False
            return True
        elif abbreviated_key == 'SOMEARROW':
            if self.actions == self.lastActions:
                return False
            return True

    def get_actions(self):

        # to be used with a loop that read all the current actions

        for event in pygame.event.get():  # User did something.
            if event.type == pygame.QUIT:  # If user clicked close.
                done = True  # Flag that we are done so we exit this loop.
            elif event.type == pygame.JOYBUTTONDOWN:

                    # print("Joystick button pressed.")

                pass
            elif event.type == pygame.JOYBUTTONUP:

                    # print("Joystick button released.")

                pass
        

        for count in range(self.joystick_count):
            
            joystick = pygame.joystick.Joystick(count)
            joystick.init()
            print("****************************",joystick.get_name(),"ID : ", joystick.get_id())
            name = joystick.get_name()

                    # Usually axis run in pairs, up/down for one, and left/right for
                    # the other.

            axes = joystick.get_numaxes()
            for i in range(axes):  # each axis is a different topic
                axis_value = joystick.get_axis(i)
                if i == 0:

                        # -----left_joy   controls
                        #     LEFT  RIGHT
                        #     -1     1

                    self.actions[('controller_' + str(count), 'joystick', 'left', 'horizontal')] = \
                        round(axis_value, 4)
                if i == 1:

                        # -----left_joy   controls
                        #     UP     DOWN
                        #     -1     1

                    self.actions[('controller_' + str(count), 'joystick', 'left', 'vertical')] = \
                        round(axis_value, 4)
                if i == 2:

                        # -----left_joy   controls
                        #     LEFT  RIGHT
                        #     -1     1

                    self.actions[('controller_' + str(count), 'joystick', 'right', 'horizontal')] = \
                        round(axis_value, 4)
                if i == 3:

                        # -----right_joy   controls
                        #     UP     DOWN
                        #     -1     1

                    self.actions[('controller_' + str(count), 'joystick', 'right', 'vertical')] = \
                        round(axis_value, 4)
            buttons = joystick.get_numbuttons()
            for i in range(buttons):
                button_value = joystick.get_button(i)
                self.actions[('controller_' + str(count), 'button', i)] = button_value
            hats = joystick.get_numhats()

                    # Hat position. All or nothing for direction, not a float like
                    # get_axis(). Position is a tuple of int values (x, y).

            for i in range(hats):
                hat_value = joystick.get_hat(i)
                self.actions[('controller_' + str(count), 'arrow', i)] = hat_value
            
        return self.actions


if __name__ == '__main__':

    # left joystick motor forward left right reverse
    # right joystick only tilt forward and backward
    # right top bumper straight up , right bottom bumper straight down

    Controller()
