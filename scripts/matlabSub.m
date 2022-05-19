clear; clc;
rosinit;

jointSub=rossubscriber('/dynamixel_workbench/joint_states','sensor_msgs/JointState'); %Creacion del suscriptor
jointSub.LatestMessage.Position

%%
rosshutdown;