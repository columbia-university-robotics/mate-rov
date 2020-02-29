#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Columbia University Robotics Club
# MATE-ROS Competition
# (c) Columbia University Robotics Club, 2019 - 2020.
# All Rights Reserved. Development by the Software Team
# Contacts: Jordan P, jmp2291@columbia.edu ; Austin T, alt2177@columbia.edu
#

import pygame

class Pygame_Controller(object):
	controller_count = 0
	controller_list = []

	def __init__(self):
		pygame.init()
		pygame.joystick.init()
		self.controller_count = pygame.joystick.get_count()
		for i in range(self.controller_count):
			controller = pygame.joystick.Joystick(i)
			controller.init()
			self.controller_list.append(controller)

	def get_events(self):
		return pygame.event.get()

	def get_quit(self):
		return pygame.QUIT
		

