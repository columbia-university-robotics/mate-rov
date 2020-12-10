# MATE ROV Simulation

This is a simply barebones simulation of the MATE ROV to be used for design and path planning purposes. The model was created using basic Newtonian Mechanics.



## MATE ROV BASE Class

This class will house the main Mate ROV file that can be inherited into different designs. The orientations of the propellers are set in standard [Spherical coordinates](https://en.wikipedia.org/wiki/Spherical_coordinate_system), we can set up a local coordinate system at the origin of the MATE ROV model. The class needs to be fed some data points shown below:

* numPropellors: Holds the number of propellers
* r: Holds the distance each propeller is from the center of the ROV
* thetaArm: Holds the theta angle for the Arm of the propeller->Angle from the z-axis
* phiArm: Holds the phi angle for Arm of the Propeller->Angle from the x-axis
* thetaProp: Holds the theta for the propeller->Angle from the z-axis
* phiProp: Holds the phi angle for each Propeller->Angle from the x-axis
* mass: Holds the mass in kg

Note: The thetaProp and phiProp measurements should be written with respect to the coordinate system of the propeller. Where $\theta,\phi = 0$ is pointing ahead, whatever forward is for the ROV.



## Propeller Class

This is a very barebones class that the other MATE ROV class can utilize to quickly calculate the the forces produced by any number of propellers in the design being tests. The class needs to be fed the following:

* pwm_min: is the minimum PWM that the propeller can take. By default it is set to 1100
* pwm_max: is the maximum PWM the the propeller can take. By default it is set to 1900
* mass: the mass of the object in kg. By default it is set to 156g

The thrust for a propeller can be modeled using the following equation:

$$F_T=(2.0 E -6(p)^2 -0.004(p) - 3.9195)*9.80665$$

Where $F_t$ is the thrust produced by a propeller and $p$ is the PWM given. The thrust was modeled using the 10V dataset given by [Blue Robotics](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/). The model above gave an $R^2$ of 0.97, which is definitely very good for our use case. We multiply by 9.80665 since the thrust is given in KgF, this will convert to Newtons which is a much more useful unit for our needs.