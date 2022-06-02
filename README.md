# Lab 3: Phantom X inverse kinematics with ROS on python 
By: Jhon Brandol Muñoz Romero and Nicolas Gil Rojas

## Abstract

- - - 
##  Workspace of the robot - MATLAB + Toolbox

To get and know the workspace of the robot we used forward kinematics giving us a 2D view of many reachable points by the robot.

<a href="https://ibb.co/sVgrZWY"><img src="https://i.ibb.co/BCfR9nM/Workspace.png" alt="Workspace" border="0"></a>

## Analysis:
-Knowing that the Phantom X robot has 4 DOFs, of which 3 correspond to position, the remaining DOF
provides an independent measure for an orientation angle (assume orientation at fixed angles).
What orientation angle is it?

-How many possible solutions exist for the inverse kinematics of the Phantom X manipulator?

-See what the dexterous space of a manipulator consists of.

## How to use this repo
Clone this repo onto your Catkin workspace. Also is needed Matlab with dynamixel messages.

With the Phantom X robot connected into your system you have to identify de IDs of the motors and modify them on the _config/joints.yaml_ file. In our case our robot had the ID from 6 to 10.

Compile the package using the following command in a terminal on catkin workspace

`caktin build lab3rob22_1`

Then, source bash 

`source devel/setup.bash`

After this is done and with no errors showed you will be ready to run de scripts.

On the scripts folder there will be a python script named _PickNDrop.py_ and .

# Python Script

The python script can be run using the following commands, on one terminal run:

`roslaunch lab3rob22_1 px_controllers.launch`

on other sourced terminal run:

`rosrun lab3rob22_1 PickNDrop.py`

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

### ROS - Pick and place application:
The objective is to implement a Pick And Place application with the Phantom X robot, which consists
in the following. 
The robot must take the type 1 piece that is on its right and place it in front.

<a href="https://imgbb.com/"><img src="https://i.ibb.co/nf59ZPk/Screenshot-from-2022-06-01-20-59-24.png" alt="Screenshot-from-2022-06-01-20-59-24" border="0"></a>

- - -
## Conclusions
- In this lab we learned how to command the robot joints and get their position using both, python and matlab.

- The python script can be upgraded so we get the current position of the joints and let the joint move gradualy from that position.

- Be careful when starting the robot's motors, as this can be dangerous.
