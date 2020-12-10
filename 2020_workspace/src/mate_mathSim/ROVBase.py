impprt Propeller as prop
class ROVBase():
    '''
    This class will house the main Mate ROV file that can be inherited into different designs

    The orientations of the propellors are set in standard Spherical coordinates.
    https://en.wikipedia.org/wiki/Spherical_coordinate_system

    The init will take the following

    numPropellors=0#Holds the number of propellors
    r=[]# holds the distance each propeller is from the center of the ROV
    thetaArm=[]#Holds the theta angle for the Arm of the propellor:Angle from the z-axis
    phiArm=[]#holds the phi angle for Arm of the Propellor:Angle from the x-axis
    thetaProp=[]#Holds the theta for the propellor:Angle from the z-axis
    phiProp=[]#holds the phi angle for each Propeller:Angle from the x-axis



    Note: The theta and phi measurements should be written with respect to the
    coordinate system of the propellor. Where theta,phi = 0 is pointing ahead, whatever
    forward is for the ROV.

    '''
    numPropellors=0#Holds the number of propellors
    r=[]# holds the distance each propeller is from the center of the ROV
    thetaArm=[]#Holds the theta angle for the Arm of the propellor:Angle from the z-axis
    phiArm=[]#holds the phi angle for Arm of the Propellor:Angle from the x-axis
    thetaProp=[]#Holds the theta for the propellor:Angle from the z-axis
    phiProp=[]#holds the phi angle for each Propeller:Angle from the x-axis
    mass=0.0#Holds the mass of the MATE ROV
    propellers=[]#list of propellers

    def __init__(self,numPropellors,r,thetaP,phiP,thetaA,phiA,mass=10):
        self.numPropellors=numPropellors
        self.r=r
        self.thetaArm=thetaA
        self.phiArm=phiA
        self.thetaProp=thetaP
        self.phiProp=phiP

    #TODO:Change the pwm of a single propeller
    #TODO:Change the pwm of all the propellers
    #TODO:Change the angle of all the propellers
    #TODO:Change the angle of a single propeller


    #TODO:Calculate torques in the phi,theta directions

    #TODO:Calculate forces in the x,y,and z directions


    #TODO:Simulate










    def createPropellors():
        #this will create the propellors we need
        for i in range(1,self.numPropellors):
            self.propellers.append(prop())
