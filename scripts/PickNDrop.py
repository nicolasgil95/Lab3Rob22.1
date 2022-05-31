#!/usr/bin/env python
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

q=np.array([0.0, 0.0, 0.0, 0.0, 0.0])

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
        value=int((ang+135)*np.pi/(180*270/1024)) #conversion de angulos Deg a bits
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,art_ID,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def inv_kin(MTH):
       #WristPos=MTH[0:3,3]-l[3]*MTH[0:3,2]
    #XYPos=np.sqrt(WristPos[0]**2+WristPos[1]**2)
    #Z=WristPos[2]-l[0]
    #R=np.sqrt(XYPos**2+Z**2)
    #theta3=np.arccos((num)/(den))
    #theta2=np.arctan2(Z,R) + np.arctan2(l[2]*np.sin(theta3),l[1]+l[2]*np.cos(theta3))
    np.set_printoptions(suppress=True)
    T = MTH
    Tw = T-(l[3]*T[0:4,2]).reshape(4,1)
    q0 = np.arctan2(Tw[1,3],Tw[0,3])
    # Solucion 2R
    h = Tw[2,3] - l[0]
    r = np.sqrt(Tw[0,3]**2 + Tw[1,3]**2)
    theta3 = np.arccos((r**2+h**2-l[1]**2-l[2]**2)/(2*l[1]*l[2]))
    theta2 = np.arctan2(h,r) + np.arctan2(l[2]*np.sin(theta3),l[1]+l[2]*np.cos(theta3))
    q1=-(np.pi/2-theta2)
    q2=-theta3
    Rp = (rotz(q1).T).dot(T[0:3,0:3])
    pitch = np.arctan2(Rp[2,0],Rp[0,0])
    q3=pitch-q1-q2
    q = np.array([q0,q1,q2,q3])
    return q

HOME=PX.fkine(Home[0:4])
Izq=transl(0,15,15)@troty(np.pi/2)@trotx(-np.pi/2)
Der=(transl(0,-18,0)@trotz(np.pi/2)@troty(np.pi))
PosIzq=Izq@transl(-30,0,0)
PosDer=(Der@transl(0,0,-35))
front=transl(25,0,5)
Posfront=(front@transl(0,0,30))

#Home-Posfront-PosIzq-Izq-PosIzq-Posfront-front-Posfront-PosDer-Der-PosDer-Posfront-Front-Posfront-Home


if __name__ == '__main__':
    #llevar a home
    for i in range(len(motorID)):
        moveart('', motorID[i], 'Goal_Position', Home[i], 0)
    #llevar a pos1
    sol=inv_kin(Izq)
    print(sol)
    for i in range(len(motorID)):
        moveart('', motorID[i], 'Goal_Position', sol[i], 0)
    #llevar a pos2
    sol=inv_kin(PosIzq)
    print(sol)
    for i in range(len(motorID)):
        moveart('', motorID[i], 'Goal_Position', sol[i], 0)
    #llevar a pos3
    sol=inv_kin(Izq)
    print(sol)
    for i in range(len(motorID)):
        moveart('', motorID[i], 'Goal_Position', sol[i], 0)
    #llevar a pos4
    sol=inv_kin(Izq)
    print(sol)
    for i in range(len(motorID)):
        moveart('', motorID[i], 'Goal_Position', sol[i], 0)
    #llevar a pos5
    sol=inv_kin(Izq)
    print(sol)
    for i in range(len(motorID)):
        moveart('', motorID[i], 'Goal_Position', sol[i], 0)
    #llevar a pos6
    sol=inv_kin(Izq)
    print(sol)
    for i in range(len(motorID)):
        moveart('', motorID[i], 'Goal_Position', sol[i], 0)
