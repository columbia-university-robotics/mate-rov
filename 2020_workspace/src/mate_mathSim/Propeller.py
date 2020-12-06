import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

class Propeller():
    def __init__(self,pwm_min=1100,pwm_max=1900,m=0.156):
        """
            @xyz : base location of prop. relative to the center of the submarine.
            @uvw : is a direction vector which corresponds to forward
            @input_min : the min PWM
            @input_max : the max PWM
        """
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        #self.dragC=b #Holds the drag coefficient
        self.m=m

    def in_limits(self,t,pwm):
        pass

    def setPWM(self,pwm):

        if pwm < self.pwm_min:
            #We want to notify the user and just set it to the min
            print("Your pwm is too Low:The min is {}!".format(self.pwm_min))
            self.pwm=self.pwm_min
        else if pwm > self.pwm_max:
            #We again want to notify the user and set it to max
            print("Your pwm is too High:The max is {}!".format(self.pwm_max))
            self.pwm=self.pwm_max
        self.pwm=pwm

    def getAcceleration(self,t,y):
        #The data will pre presented in a 2d matrix [velocity] We can expand this later

        #This is the result from a regression analysis of the thruster data given in
        #https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/
        #The equation:Thrust=2.0 E -6(PWM)^2 -0.004(PWM) - 3.9195
        #Had an R^2 of 0.971, which is more than enough for our purposes
        #This was taken from the 10V dataset

        #pwm is in micro Seconds

        thrust = (2.0 * 10**(-6) * self.pwm**2 -0.004*self.pwm -3.9195)*9.80665

        return (1/self.m)*(thrust)




    def simulate(self):
        #This is a test method that will model the motion of a single propellor
        #it will be modeled as a symbol

        self.setPWM(1100)#Set the PWM
        #self.dragC=1
        t=np.arange(0,10,0.1)
        sol=solve_ivp(self.getAcceleration,[0,1000],[0],t_eval=t)
        t=sol.t
        vel=sol.y[0]
        plt.plot(t,vel)
        plt.show()

if __name__ == '__main__':
    p= Propeller(1,0)
    p.simulate()
