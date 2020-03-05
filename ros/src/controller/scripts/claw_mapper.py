#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Columbia University Robotics Club
# MATE-ROS Competition
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Software Team
# Contacts: Jordan P, jmp2291@columbia.edu ; Austin T, alt2177@columbia.edu
#

class Claw_Mapper(object):

	def __init__(self, publish, create_publisher):
		
		self.name = "claw"							
		self.publish = publish                          #passing the publish function from controller class through this class

		self.mapping_dict = {}
		
		self.mapping_dict["left_vert"] = create_publisher('controller/claw/left_vert')
		self.mapping_dict["left_hori"] = create_publisher('controller/claw/left_hori')
		self.mapping_dict["right_vert"] = create_publisher('controller/claw/right_vert')
		self.mapping_dict["right_hori"] = create_publisher('controller/claw/right_hori')
		self.mapping_dict["right_topbump"] = create_publisher('controller/claw/right_topbumper')
		self.mapping_dict["right_botbump"] = create_publisher('controller/claw/right_bottombumper')
		

	def map(self, button_pressed, value): 				#button_pressed specifies the button, value specifies the value (e.g. 0.75)
														
		if (button_pressed in self.mapping_dict):

			self.publish(self.mapping_dict[button_pressed], value)


