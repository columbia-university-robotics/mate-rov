
# WELCOME !


### Installation
The first thing to realize is that there are different ROS versions.
```
http://wiki.ros.org/ROS/Installation
```
We'll be using "melodic". So click there and follow their setup instructions. Please reach out if you get stuck anywhere.
That means if you ever see "kinetic"/"indigo" anywhere on a tutorial you can easily replace it with "melodic" and it will work!!!

### Sending Messages Between ROS "nodes"
Great now let's learn a fundamental part of ROS.
```
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
```
* NOTES : any time you see a "$" sign in the tutorials you can be sure that the text should go into your computers terminal/ command line. The short-cut to open a terminal on a linux is ctrl+alt+t. Press those buttons and a terminal will open.

### The Drone We'll Modify
OK! great you made it this far, this means you understand the basics to have different parts of our program interact with each other. 

Besides getting a better understanding of using the commandl line, python and c++ we'll also get a chance to use git.
For this next portion what we'll be doing is actually getting into the heart of what we might use to simulate the "submarine" we'll use for the MATE ROV challenge.

heres a link to the git repo https://github.com/NishanthARao/ROS-Quadcopter-Simulation

For simplicity we'll be working from your home directory ( i.e. "~/").

Open a terminal using the method you used before. And input the following ( line-by-line).
```
sudo apt-get update
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt-get install ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers
cd ~/
mkdir -p quad_simulation_workspace/src
cd quad_simulation_workspace/src
git clone https://github.com/NishanthARao/ROS-Quadcopter-Simulation.git
cd ./ROS-Quadcopter-Simulation/src/
chmod u+x control.py
chmod u+x pid.py
cd ../../../
# make sure the following is sourced
source /opt/ros/melodic/setup.bash 
# now the fun begins, lets hope everything builds correctly! 
catkin_make
# if everything built correctly you should now be able to run the following
source devel/setup.bash
roslaunch fly_bot Kwad_gazebo.launch
```
:) 
Ok great at this point if everything thing went well then you should see the gazebo simulation of the drone!
If you get an error and after reading the longwinded ROS error message you feel lost, please reach out with  screen shot of the screen.

Ok great now for lets make the drone hover using their python node which uses subcribers and publishers!
Leave the last terminal as is and open another terminal.
```
cd ~/quad_simulation_workspace
source /opt/ros/melodic/setup.bash 
source devel/setup.bash
rosrun fly_bot control.py
```
### mini-TUTORIAL COMPLETED !
Voila! :) enjoy the fruits of your labor as we will modify the ROS package you got AND launched from git in order to simulate an underwater vehicle!

I'm sure I'll be able to recommend / you'll find more tutorials useful as we continue onwards but please take this moment to feel proud of yourself!

### Now I ask you to notice a directory called "example_workspace"
That directory is meant as a guide to building your own workspace.
Please remember to catkin_make it from the workspaces root directory,
And then source its devel/setup.bash file.

