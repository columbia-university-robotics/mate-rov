#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Columbia University Robotics Club
# MATE-ROS Competition
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Software Team
# Based on code written by Jonathan 
# Contacts: Jordan P, jmp2291@columbia.edu ; Austin T, alt2177@columbia.edu
#

import rospy
from std_msgs.msg import Float32
from driver_mapper import Driver_Mapper
from claw_mapper import Claw_Mapper
from pygame_class import Pygame_Controller


class Controller(object):

	def __init__(self):

		mapping_list = []
		mapping_list.append(Driver_Mapper(self.publish, self.create_publisher))			#rov has mapping index 0 because it was created first
		mapping_list.append(Claw_Mapper(self.publish, self.create_publisher))				#we could easily switch it so that the claw is index 0

		pygame_controller = Pygame_Controller()

		mapping_index = []
		
		pressed_start = []
		
		for i in range(pygame_controller.controller_count):
			mapping_index.append(0)
			pressed_start.append(0)

		rospy.init_node("controller")

		r = rospy.Rate(10)  	# 10hz

		axisDict = {}									#for any of these 3 dictionaries, if we want to add more values, we can
		axisDict[0] = "left_hori"
		axisDict[1] = "left_vert"
		axisDict[2] = "right_hori"
		axisDict[3] = "right_vert"

		buttonDict = {} 
		buttonDict[5] = "right_topbump"
		buttonDict[7] = "right_botbump"

		hatDict = {}									#we currently do not use the hats, so this dictionary is blank

		done = False

		while not rospy.is_shutdown() and not done:

			for event in pygame_controller.get_events():
				if event.type == pygame_controller.get_quit():
					done = True
					break
			
			for i in range(pygame_controller.controller_count):				#this loops through all our controllers 
				current = pygame_controller.controller_list[i]

				if current.get_button(9) == 1 and pressed_start[i] == 0:	#allowing us to toggle controller between rov and claw
					mapping_index[i] = (mapping_index[i] + 1) % len(mapping_list)

				pressed_start[i] = current.get_button(9)		#prevent us from toggling too many times (e.g. holding down button 9)
				


				current_mapping = mapping_list[mapping_index[i]]	#connecting mapping_list to mapping_index		

				for axisIn in range(current.get_numaxes()):			#these loops test if these axes, buttons, and hats are ones we use
					if axisIn in axisDict:
						axis_value = current.get_axis(axisIn)
						current_mapping.map(axisDict[axisIn], axis_value)

				for butIn in range(current.get_numbuttons()):
					if butIn in buttonDict:
						button_value = current.get_button(butIn)
						current_mapping.map(buttonDict[butIn], button_value)

				for hatIn in range(current.get_numhats()):
					if hatIn in hatDict:
						hat_value = current.get_hat(hatIn)
						current_mapping.map(hatDict[hatIn], hat_value)



			r.sleep()
		

	def publish(self, publisher, value):
		publisher.publish(value)

	def create_publisher(self, topic, data_type = Float32, queue_size = 10):
		return rospy.Publisher(topic, data_type, queue_size = queue_size)



if __name__ == '__main__':

	Controller()
