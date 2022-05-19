# Lab 2: PX manipulation with ROS on python and matlab

By: Jhon Brandol Muñoz Romero and Nicolas Gil Rojas

_We had a trouble doing the commits to this repo so all were made by Nicolas Gil using VS code but Jhon Muñoz participated in the creation of the scrips and the explanation of matlab code, he made pull request but I wasn't able to merge them._

## Abstract
In this lab we try to manipulate the Phantom X robot using the keyboard of our laptop. Also we give a position to the robot using Matlab and get a representation of the current pose using the RVC toolbox.
- - - 
##  Cinematic analysis of the robot
The first one, reference systems are assigned according to the DHstd convention. In this case, you can si the Root Phantom that is plotted with the tool in Matlab. 

<a href="https://imgbb.com/"><img src="https://i.ibb.co/tXtr8yM/pp.jpg" alt="pp" border="0"></a>

Then, there are the convention parameters of DHstd, and it is represented in the following table, where the letter q is variable.Therefore, the measurement is given in centimeters.

 <a href="https://imgbb.com/"><img src="https://i.ibb.co/482tP1q/para.png" alt="para" border="0"></a>

## How to use this repo
Clone this repo onto your Catkin workspace. Also is needed Matlab with dynamixel messages.

With the Phantom X robot connected into your system you have to identify de IDs of the motors and modify them on the _config/joints.yaml_ file. In our case our robot had the ID from 6 to 10.

Compile the package using the following command in a terminal on catkin workspace

`caktin build lab2rob22_1`

Then, source bash 

`source devel/setup.bash`

After this is done and with no errors showed you will be ready to run de scripts.

On the scripts folder there will be a python script named _KeyMove.py_ and 4 matlab scripts: *PX_Robot.m*, _matlabPub.m_, _matlabSub.m_ and _matlabConnectionROSnPX.m_.

# Python Script

The python script can be run using the following commands, on one terminal run:

`roslaunch lab2rob22_1 px_controllers.launch`

on other sourced terminal run:

`rosrun lab2rob22_1 KeyMove.py`

The first command wil enable the communication with the robot, the second one will launch the python script.

After this you should be able to control the robot using you laptop's keyboard:

- Using _W_ and _S_ you will move between joints, it will work cyclical.

- Using _A_ and _D_ the choosen joint will move 90 degrees to left or right.

- Using _R_ the choosen joint will go home.

- Using _H_ the robot will go home.

Also, if you run the next command in other terminal you will visualize the robot in rviz:

`roslaunch lab2rob22_1 px_rviz_dyna.launch`

This is how it'll see on your pc's screen

<a href="https://ibb.co/nDsby4Z"><img src="https://i.ibb.co/jLvTYqB/rvix-Python.png" alt="rvix-Python" border="0"></a>

You will see the video of the robot moving with the keyboard [here](https://youtu.be/rZpshr-DT9Q).

The script is simple: first we import some libraries to python.

<a href="https://imgbb.com/"><img src="https://i.ibb.co/ccmTfWY/LIB.png" alt="LIB" border="0"></a>

Then, there are the definitions of the functions used, one to communicate and send messages to the command service, the second and third ones aren't used as they are the required to connect to the topic and get the current position, but, sadly, we didn't manage to used that current position. Finally, there are two functions to move between joints.

The service message function receives degrees angles and convert them to bits so the message understands them. Thats done by getting the desired angle, add 135 to it, and divide into the resolution of movement.

<a href="https://ibb.co/nQc3zSL"><img src="https://i.ibb.co/c2FbDdr/Func.png" alt="Func" border="0"></a>

In the main you'll find the call for the functions. Also there is a part of the code that will give each motor a torque value so the robot won't move very aggresive.

<a href="https://ibb.co/0mD587B"><img src="https://i.ibb.co/JmBZ01x/main.png" alt="main" border="0"></a>

Again, after running this script the terminal will be bugged and won't show any text.

- - -

### Using the MATLAB script
This is the [link](https://youtu.be/wtryydCzOSE) to watch the video:


First of all, for using both of the scripts you will need to start a ROS master node To do that we need two terminal sessions. 

We use the "link" function of robotic toolbox 

<a href="https://ibb.co/k8rHtFh"><img src="https://i.ibb.co/BZ7z9Rg/M1.png" alt="M1" border="0"></a>

On the other hand, Ros and the topic have been connected. Given the above, it calls you the services and the messages to make the robot work.

<a href="https://ibb.co/YpQpRsh"><img src="https://i.ibb.co/NZCZVBK/M2.png" alt="M2" border="0"></a>

Robot at home

<a href="https://ibb.co/KjLkTC2"><img src="https://i.ibb.co/nsnvxh7/H.jpg" alt="H" border="0"></a>

You can see some of the robot configurations in the next image

<a href="https://ibb.co/4pyNcKp"><img src="https://i.ibb.co/qpG1z7p/f1.jpg" alt="f1" border="0"></a>

Additionally, we can watch another position about the tool of work 

<a href="https://ibb.co/VYHPFTV"><img src="https://i.ibb.co/Mk83z7n/f2.jpg" alt="f2" border="0"></a>

- - -
## Conclusions
- In this lab we learned how to command the robot joints and get their position using both, python and matlab.

- The python script can be upgraded so we get the current position of the joints and let the joint move gradualy from that position.

- Be careful when starting the robot's motors, as this can be dangerous.