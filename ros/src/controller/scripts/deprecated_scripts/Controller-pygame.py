# -*- coding: utf-8 -*-
"""
Created on Fri Oct 11 20:49:21 2019

@author: https://www.pygame.org/docs/ref/joystick.html#pygame.joystick.init

Sample code from pygame.org



"""

import pygame
import numpy as np
import rospy

# Define some colors.
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')


# This is a simple class that will help us print to the screen.
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


pygame.init()

# Set the width and height of the screen (width, height).
screen = pygame.display.set_mode((500, 700))

pygame.display.set_caption("My Game")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates.
clock = pygame.time.Clock()

# Initialize the joysticks.
pygame.joystick.init()

# Get ready to print.
textPrint = TextPrint()
dimensions = 25
emptyChar = "-"
drawChar = "+"
arx = np.chararray((dimensions,dimensions), unicode=True , offset = 3)   # ( rows , columns )
arx.fill(emptyChar)
arx[ np.size(arx,0)//2 , np.size(arx,1)//2 ] = "*"
currentPosition = [ np.size(arx,0)//2 , np.size(arx,1)//2 ]
def update_Arx( x = 0, y = 0):
    global arx
    global drawChar
    global emptyChar
    global arx
    global dimensions
    global currentPosition
    x = round(x,1)
    y = round(y,1)
    
    
    if( round(x,1) > 0.5 ):
        if( currentPosition[1] < dimensions-1 ):
            currentPosition[1] += 1
    elif( round(x,1) < -0.5 ):
        if( currentPosition[1] > 0 ):
            currentPosition[1] -= 1
    if( round(y,1) > 0.5 ):
        if( currentPosition[0] < dimensions-1 ):
            currentPosition[0] += 1
    elif( round(y,1) < -0.5 ):
        if( currentPosition[0] > 0 ):
            currentPosition[0] -= 1
    if(arx[ currentPosition[0] , currentPosition[1] ] != drawChar ):
        arx[ currentPosition[0] , currentPosition[1] ] = drawChar
    
# -------- Main Program Loop -----------
while not done:
    #
    # EVENT PROCESSING STEP
    #
    # Possible joystick actions: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
    # JOYBUTTONUP, JOYHATMOTION
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
        elif event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

    #
    # DRAWING STEP
    #
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()

    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()

    textPrint.tprint(screen, "Number of joysticks: {}".format(joystick_count))
    textPrint.indent()

    # For each joystick:
    ar = np.zeros((3,3))   # ( rows , columns )
    ar2 = np.zeros((3,3))   # ( rows , columns )

    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        
        textPrint.tprint(screen, "Joystick {}".format(i))
        textPrint.indent()

        # Get the name from the OS for the controller/joystick.
        name = joystick.get_name()
        textPrint.tprint(screen, "Joystick name: {}".format(name))

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        textPrint.tprint(screen, "Number of axes: {}".format(axes))
        textPrint.indent()
        """
                        Human readable version
JOYSTICK:       LEFT                              RIGHT
            axis 0 and 1                      axis 3 and 4
Movement:      
                -1.ax1                            -1.ax3
                 
         -1.ax0         1.ax0           -1.ax4             1.ax4

                 1.ax1                             1.ax3
      
        """
        for i in range(axes):
            axis = joystick.get_axis(i)
            if( i == 0):
            #-----left_joy
            #      controls
            #     LEFT  RIGHT
            #     -1     1
                if( round(axis,6) < 0 ): #LEFT
                    ar[1,0] = round(axis,1)
                elif( round(axis,6) > 0 ): #RIGHT
                    ar[1,2] = round(axis,1)
                update_Arx( x = axis, y = 0)
                
		# =======================================
		# publish axis -- left joystick horizontal

            if( i == 1):
            #-----left_joy
            #      controls
            #     UP     DOWN
            #     -1     1
                if( round(axis,6) < 0 ): # UP
                    ar[0,1] = round(axis,1)
                elif( round(axis,6) > 0 ): #DOWN
                    ar[2,1] = round(axis,1) 
                update_Arx( x = 0, y = axis)

		# =======================================
		# publish axis -- left joystick horizontal

            if( i == 3):
            #-----right_joy
            #      controls
            #     UP     DOWN
            #     -1     1
                if( round(axis,6) < 0 ): #UP
                    ar2[0,1] = round(axis,1)
                elif( round(axis,6) > 0 ): #DOWN
                    ar2[2,1] = round(axis,1)   

		# =======================================
		# publish axis -- left joystick horizontal
         
            if( i == 4):
            #-----left_joy
            #      controls
            #     LEFT  RIGHT
            #     -1     1
                if( round(axis,6) < 0 ): #LEFT
                    ar2[1,0] = round(axis,1)
                elif( round(axis,6) > 0 ): #RIGHT
                    ar2[1,2] = round(axis,1)

		# =======================================
		# publish axis -- left joystick horizontal

                textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(i, axis))
        
	textPrint.unindent()
        textPrint.tprint(screen, "")
        textPrint.tprint(screen, "           LEFT                                         RIGHT")
        textPrint.tprint(screen, "{:6.1f}  {:6.1f}  {:6.1f}                        {:6.1f}  {:6.1f}  {:6.1f}".format( ar[0,0] , ar[0,1] , ar[0,2] , ar2[0,0] , ar2[0,1] , ar2[0,2] ))
        textPrint.tprint(screen, "{:6.1f}  {:6.1f}  {:6.1f}                        {:6.1f}  {:6.1f}  {:6.1f}".format( ar[1,0] , ar[1,1] , ar[1,2] , ar2[1,0] , ar2[1,1] , ar2[1,2]))
        textPrint.tprint(screen, "{:6.1f}  {:6.1f}  {:6.1f}                        {:6.1f}  {:6.1f}  {:6.1f}".format( ar[2,0] , ar[2,1] , ar[2,2] , ar2[2,0] , ar2[2,1] , ar2[2,2]))
        
        
        for j in range(np.size(arx , 0)):
            rowString = ""
            for k in range(np.size(arx , 1)):
                if( arx[j,k] == emptyChar ):
                    rowString += " " +")("+ " "
                else:
                    rowString += " " + arx[j,k] + " "
            textPrint.tprint(screen, rowString)
        
        
        
        """
################################################################################
#########                 __INIT__ BUTTONS                       ################
######### THIS PART CONTROLS THE REST OF THE CONTROLLER BUTTONS ################
################################################################################
        buttons = joystick.get_numbuttons()
        textPrint.tprint(screen, "Number of buttons: {}".format(buttons))
        textPrint.indent()

        for i in range(buttons):
            button = joystick.get_button(i)
            textPrint.tprint(screen,
                             "Button {:>2} value: {}".format(i, button))
        textPrint.unindent()
        hats = joystick.get_numhats()
        textPrint.tprint(screen, "Number of hats: {}".format(hats))
        textPrint.indent()

        # Hat position. All or nothing for direction, not a float like
        # get_axis(). Position is a tuple of int values (x, y).
        for i in range(hats):
            hat = joystick.get_hat(i)
            textPrint.tprint(screen, "Hat {} value: {}".format(i, str(hat)))
        textPrint.unindent()

        textPrint.unindent()
################################################################################
#########                     END BUTTONS                       ################
################################################################################
        """

    #
    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    #

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 20 frames per second.
    clock.tick(20)

# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()
