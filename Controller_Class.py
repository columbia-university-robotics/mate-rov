




import pygame
#import rospy
#from std_msgs.msg import String

class Controller(object):
	#controller = None
	#axis_data = None
	#button_data = None
	#hat_data = None
	#joystick_count = None
	

	def __init__(self):
		pygame.init()
		pygame.joystick.init()

		self.actions = {}
    		self.joystick_count = pygame.joystick.get_count()

	def get_actions( self ):
		# to be used with a loop that read all the current actions
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                
                # Get the name from the OS for the controller/joystick.
                name = joystick.get_name()
                # Usually axis run in pairs, up/down for one, and left/right for
                # the other.
                axes = joystick.get_numaxes()
                for i in range(axes):   # each axis is a different topic
                    axis_value = joystick.get_axis(i)
                    if( i == 0):
                    #-----left_joy   controls
                    #     LEFT  RIGHT
                    #     -1     1
                        self.actions[("joystick", "left" , "horizontal")] = round(axis_value,1)
                    if( i == 1):
                    #-----left_joy   controls
                    #     UP     DOWN
                    #     -1     1
                        self.actions[ ("joystick", "left" , "vertical") ] = round(axis_value,1)
                    if( i == 2):
                    #-----left_joy   controls
                    #     LEFT  RIGHT
                    #     -1     1
                        self.actions[ ("joystick", "right" , "horizontal") ] = round(axis_value,1)
                    if( i == 3):
                    #-----right_joy   controls
                    #     UP     DOWN
                    #     -1     1
                        self.actions[ ("joystick", "right" , "vertical") ] = round(axis_value,1)           
                buttons = joystick.get_numbuttons()
                for i in range(buttons):
                    button_value = joystick.get_button(i)
                    self.actions[ ("button",i) ] = button_value 
                hats = joystick.get_numhats()
                # Hat position. All or nothing for direction, not a float like
                # get_axis(). Position is a tuple of int values (x, y).
                for i in range(hats):
                    hat_value = joystick.get_hat(i)
                    self.actions[ ("arrow",i) ] = hat_value 
		return self.actions




if __name__ == "__main__":

	# left joystick motor forward left right reverse
	# right joystick only tilt forward and backward
	# right top bumper straight up , right bottom bumper straight down
	control = Controller()
	control.init()
	while( True ):
# http://wiki.ros.org/msg   #    float32
	#pub = rospy.Publisher('controller', String, queue_size=10)
	#rospy.init_node('joystick')
	#rospy.init_node('button')
	#rospy.init_node('arrow')
	r = rospy.Rate(10) # 10hz
	#while( not rospy.is_shutdown() ):
		action_dict = control.get_actions()
		
		for k,v in action_dict.items() :
			if( "joystick" in k ):
				# publish necessary joystick values
				pass
			if( "button" in k ):
				# publish necessary button values
				pass
			if( "arrow" in k ):
				# publish necessary arrow values
				pass
		#r.sleep()
















