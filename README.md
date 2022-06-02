# Lab 3: Phantom X inverse kinematics with ROS on python 
By: Jhon Brandol Muñoz Romero and Nicolas Gil Rojas

## Abstract
In this lab we learn and use inverse kinematics to move the Phantom X robot doing a simple pick and place application and some workspace moves. 
- - - 
##  Workspace of the robot
To get and know the workspace of the robot we used forward kinematics giving us a 2D view of many reachable points by the robot.

<a href="https://ibb.co/6DW5vbQ"><img src="https://i.ibb.co/7r1FgVh/Workspace.png" alt="Workspace" border="0"></a>

Also, we know there are some functions on the Peter Corke toolbox to get the inverse kinematics of a robot. Those functions are part of the class SerialLink so, to use them, we have to define a SerialLink object first. Some of these functions are _ikine_, _ikunk_, _ikcon_ or _ikine6s_ but some of them are used for 6 DoF robots.

The dexterous workspace is where the tool can get, for example, in this Phantom X robot the tool _Approach_ axis will always be aligned with the first motor angle.

## How to use this repo

Clone this repo onto your Catkin workspace.

With the Phantom X robot connected into your system you have to identify de IDs of the motors and modify them on the _config/joints.yaml_ file. In our case our robot had the ID from 1 to 5.

Compile the package using the following command in a terminal on catkin workspace

`caktin build lab3rob22_1`

Then, source bash 

`source devel/setup.bash`

After this is done and with no errors showed you will be ready to run de scripts.

On the scripts folder there will be a python script named _PickNDrop.py_ and _KeyMove.py_.

# Pick and Place Script

This is the __[link](https://youtube.com/shorts/pO0-6QPsQTQ)__ to watch the first video:
This is the __[link](https://youtube.com/shorts/02OXB1aoj3E
)__ to watch the first video:
The objective is to implement a Pick And Place application with the Phantom X robot, which consists
in the following. 
The robot must take the type 1 piece that is on its right and place it in front.

<a href="https://imgbb.com/"><img src="https://i.ibb.co/nf59ZPk/Screenshot-from-2022-06-01-20-59-24.png" alt="Screenshot-from-2022-06-01-20-59-24" border="0"></a>

<a href="https://ibb.co/m4N3b89"><img src="https://i.ibb.co/VTwsJmg/FramePos.png" alt="FramePos" border="0"></a>

This is the __[link](https://youtu.be/9BKicWuFVmo)__ to watch the video:
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

`roslaunch lab3rob22_1 px_rviz_dyna.launch`

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