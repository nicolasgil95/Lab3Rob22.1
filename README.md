# Lab 3: Phantom X inverse kinematics with ROS on python 
By: Jhon Brandol Muñoz Romero and Nicolas Gil Rojas

## Abstract
In this lab we learn and use inverse kinematics to move the Phantom X robot doing a simple pick and place application and some workspace moves. 
- - - 
##  Workspace of the robot
To get and know the workspace of the robot we used forward kinematics giving us a 2D view of many reachable points by the robot.

<a href="https://ibb.co/Y8h6yjV"><img src="https://i.ibb.co/QnMZCmx/PXworkspace.png" alt="PXworkspace" border="0"></a>

Also, we know there are some functions on the Peter Corke toolbox to get the inverse kinematics of a robot. Those functions are part of the class SerialLink so, to use them, we have to define a SerialLink object first. Some of these functions are _ikine_, _ikunk_, _ikcon_ or _ikine6s_ but some of them are used for 6 DoF robots.

The dexterous workspace is where the tool can get, for example, in this Phantom X robot the tool _Approach_ axis will always be aligned with the first motor angle.

The Phantom X robot has 6 soluions, 2 for the first or waist motor, another 2 for the shoulder motor and 2 more for the elbow motor. The wrist motor will only have 1 solution. 

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

This is the __[link](https://youtube.com/shorts/pO0-6QPsQTQ)__ to watch the first video and
this is the __[link](https://youtube.com/shorts/02OXB1aoj3E
)__ to watch the second video:
The objective is to implement a Pick And Place application with the Phantom X robot, which consists
in the following. Then,the robot must take the type 1 piece that is on its right and place it in front.
#Restrictions:

The paths must be of the pick and place type, that is, rectangular paths, movements
vertical to go up and down, and horizontal movement to move.

<a href="https://imgbb.com/"><img src="https://i.ibb.co/nf59ZPk/Screenshot-from-2022-06-01-20-59-24.png" alt="Screenshot-from-2022-06-01-20-59-24" border="0"></a>

To begin with, the calculations are made to locate the homogeneous transformation matrices for each point of the trajectory that the robot will make. Therefore, it can be seen in the following image.

<a href="https://ibb.co/m4N3b89"><img src="https://i.ibb.co/VTwsJmg/FramePos.png" alt="FramePos" border="0"></a>

Then, the trajectory is plotted with the function `rtb.jtraj(T0, T1, 50)`
where T0 and T1 are vectors of the first position that the robot performs, these measurements are in radians for each joint of the robot, which is obtained by entering the MTH in the function that calculates the inverse kinematics of the robot.

<a href="https://imgbb.com/"><img src="https://i.ibb.co/bFGKf2f/Screenshot-from-2022-06-01-21-54-46.png" alt="Screenshot-from-2022-06-01-21-54-46" border="0"></a>



- - -

### Using the KeyMove script

This script uses the same function to get the inverse kinematics of the robot, always gives an elbow up configuration. Also, we use the Joint_State topic to get the current position of the arm. After that, using the forward kinematics function of the toolbox we get the corresponding MTH for that pose. Then, the user inputs the axis of movement and the distance, _x_, _y_ and _z_ axis are available and positive and negative values are accepted.

Once the axis and distance are defined, we add the distance value in the corresponding axis to the MTH of the current pose calculated previously. With the start and end pose MTHs is used the toolbox _ctraj_ function to get the intermediate pose MTHs, this arrays are calculated every 0.5 cm. This new MTHs will be the input for our inverse kinematics *inv_kin* function. Knowing the joint values for each step we send them to the motors.

In the image you can see the output of the script, where it gets the current MTH and then ask for the axis of movement and the distance to move, the prints the destination MTH and starts to execute the movements on the robot. 

<a href="https://ibb.co/VDxbbkv"><img src="https://i.ibb.co/d6tXXHQ/KeyMove.png" alt="KeyMove" border="0"></a>

[Here](https://www.youtube.com/watch?v=9BKicWuFVmo) you can see the results of the script run. 

---

## Conclusions
- The results of our inverse kinematics function depends on the model we made of the robot, so, as there could be some error on the lenghts of each link the function won't give the correct angles for the actual robot.
- In the first video, it is observed that the robot picks up part number two perfectly, but at the end of the route when it has to drop it, it tends to have problems.
- In the second video, it is observed that the robot cannot drop part number two, since the diameter of this part is greater than the limit diameter that the robot tool reaches
- While writing this file we realiced that, for straight moves on any axis the tool MTH have to suffer a rotation as it will always be aligned to the waist motor position, luckly, for our small test movements it didn't seem to disturb but for bigger moves it should. 
- While using all the torque for the motors, and doing a small amount of steps between points, for example 3 in some of the Pick and Place application, the arm will have many inertia and the movement won't be soft and steady, meanwhile, when the steps are higher, instead of 3 we test even with 25, the movement will be more precise and beautyfull but will take longer.
