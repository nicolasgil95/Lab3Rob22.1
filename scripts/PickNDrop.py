#!/usr/bin/env python
from turtle import home
import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand


motorID=np.array([1,2,3,4,5])
Home=np.array([0,0,0,0,0])
l = np.array([14.5, 10.7, 10.7, 9])
qlims = np.array([[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4]])

q=np.array([0, 0, 0, 0, 0])

PX = rtb.DHRobot(
    [rtb.RevoluteDH(alpha=np.pi/2, d=l[0], qlim=qlims[0,:]),
    rtb.RevoluteDH(a=l[1], offset=np.pi/2, qlim=qlims[0,:]),
    rtb.RevoluteDH(a=l[2], qlim=qlims[0,:]),
    rtb.RevoluteDH(qlim=qlims[0,:])],
    name="PhantomX")
PX.tool = transl(l[3],0,0).dot(troty(np.pi/2).dot(trotz(-np.pi/2)))

def moveart(command, art_ID, addr_name, ang, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:
        value=int((ang+135)/(270/1024)) #conversion de angulos Deg a bits
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,art_ID,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def inv_kin(MTH):


if __name__ == '__main__':
    #llevar a home
    for i in range(len(motorID)):
        moveart('', motorID[i], 'Goal_Position', Home[i], 0)

    #llevar a pos1
    #llevar a pos2
    #llevar a pos3
    #llevar a pos4
    #llevar a pos3
    #llevar a pos5
    #llevar a pos6
