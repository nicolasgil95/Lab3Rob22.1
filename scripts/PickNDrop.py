#!/usr/bin/env python
import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand

i=1

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
        value=int((ang+150)/(300/1024)) #conversion de angulos Deg a bits
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,art_ID,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def inv_kin(MTH):
    WristPos=MTH[0:3,3]-l[3]*MTH[0:3,2]
    XYPos=np.sqrt(WristPos[0]**2+WristPos[1]**2)
    Z=WristPos[2]-l[0]
    R=np.sqrt(XYPos**2+Z**2)
    num=R**2+Z**2-l[1]**2-l[2]**2
    den=2*l[1]*l[2]
    theta3=np.arccos((num)/(den))
    theta2=np.arctan2(Z,R) + np.arctan2(l[2]*np.sin(theta3),l[1]+l[2]*np.cos(theta3))
    q[0]=np.arctan2(MTH[1,3],MTH[0,3])
    q[1]=-(np.pi/2-theta2)
    q[2]=-theta3
    aux=np.array(rotz(q[0]))
    RP=aux.transpose()@MTH[0:3,0:3]
    pitch=np.arctan2(RP[2,0],RP[0,0])
    q[3]=pitch-q[1]-q[2]
    return q

Izq=transl(0,15,5)@trotz(np.pi/2)@trotx(np.pi)
Der=(transl(0,-15,5)@trotz(np.pi/2)@troty(np.pi))
PosIzq=Izq@transl(0,0,-10)
PosDer=(Der@transl(0,0,-10))
front=transl(15,0,5)@troty(np.pi)
Posfront=(front@transl(0,0,-10))

#Home-Posfront-PosIzq-Izq-PosIzq-Posfront-front-Posfront-PosDer-Der-PosDer-Posfront-Front-Posfront-Home

if __name__ == '__main__':
    for index in range(len(motorID)):
        moveart('', motorID[index], 'Goal_Position', Home[index], 0)
    moveart('', motorID[4], 'Goal_Position', 512, 0)#abrir herramienta
    MTH_act=PX.fkine(Home[0:4])
    #print('Izq')
    #print(np.rad2deg(inv_kin(Izq)))
    #print('PosIzq')
    #print(np.rad2deg(inv_kin(PosIzq)))
    #print('Der')
    #print(np.rad2deg(inv_kin(Der)))
    #print('PosDer')
    #print(np.rad2deg(inv_kin(PosDer)))
    #print('front')
    #print(np.rad2deg(inv_kin(front)))
    #print('Posfront')
    #print(np.rad2deg(inv_kin(Posfront)))

    q=inv_kin(np.array(Posfront))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    q=inv_kin(np.array(PosIzq))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    q=inv_kin(np.array(Izq))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest

    moveart('', motorID[4], 'Goal_Position', 0, 0) #cerrar herramienta
    
    q=inv_kin(np.array(PosIzq))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    q=inv_kin(np.array(Posfront))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    q=inv_kin(np.array(front))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    moveart('', motorID[4], 'Goal_Position', 512, 0) #abrir herramienta

    q=inv_kin(np.array(Posfront))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    q=inv_kin(np.array(PosDer))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    q=inv_kin(np.array(Der))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    moveart('', motorID[4], 'Goal_Position', 300, 0) #cerrar herramienta

    q=inv_kin(np.array(PosDer))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    q=inv_kin(np.array(Posfront))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest
    
    q=inv_kin(np.array(front))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    MTH_act=MTH_dest

    moveart('', motorID[4], 'Goal_Position', 512, 0) #abrir herramienta
    
    q=inv_kin(np.array(Posfront))
    MTH_dest=PX.fkine(q[0:4])
    steps=rtb.ctraj(MTH_act,MTH_dest,5)
    for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
    
    for index in range(len(motorID)):
        moveart('', motorID[index], 'Goal_Position', Home[index], 0)
    