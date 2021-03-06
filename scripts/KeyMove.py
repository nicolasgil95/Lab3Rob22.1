#!/usr/bin/env python
from re import X
from turtle import distance
from matplotlib.pyplot import pause
import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand

i=1

axe='x'
dist=0.0

motorID=np.array([1,2,3,4,5])
Home=np.array([0,0,0,0,0])
l = np.array([14.5, 10.7, 10.7, 9])
qlims = np.array([[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4]])

q=np.array([0.0, 0.0, 0.0, 0.0, 0.0])
q_act=np.array([0.0, 0.0, 0.0, 0.0, 0.0])

PX = rtb.DHRobot(
    [rtb.RevoluteDH(alpha=np.pi/2, d=l[0], qlim=qlims[0,:]),
    rtb.RevoluteDH(a=l[1], offset=np.pi/2, qlim=qlims[0,:]),
    rtb.RevoluteDH(a=l[2], qlim=qlims[0,:]),
    rtb.RevoluteDH(qlim=qlims[0,:])],
    name="PhantomX")
PX.tool = transl(l[3],0,0).dot(troty(np.pi/2).dot(trotz(-np.pi/2)))

#Funcion para mover articulacion
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

def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)
    #rospy.spin()
    
def callback(data):
    #rospy.loginfo(data.position)
    q_act[0]=data.position[0]
    q_act[1]=data.position[1]
    q_act[2]=data.position[2]
    q_act[3]=data.position[3]
    
def getMov():
    global axe
    axe=input("Which axe will move:\n")
    if axe!='x'and axe!='y' and axe!='z':
        print("Axe doesn't exist")
        return
    global dist
    dist=float(input("How many cm travel: \n"))
    print(f'transl{axe} {dist} cm')
    return

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


if __name__ == '__main__':
    #obtener estado actual
    listener()
    pause(1)
    #obtener pose actual
    MTH_act=PX.fkine(q_act[0:4])
    MTH_dest=MTH_act
    print(MTH_act)
    while i==1:    
        #obtener eje y distancia de mov
        getMov()
        #Calcular MTH siguiente
        #print(MTH_act)
        if axe=='x':
            MTH_dest=SE3(transl(dist,0,0))@MTH_dest
        elif axe=='y':
            MTH_dest=SE3(transl(0,dist,0))@MTH_dest
        elif axe=='z':
            MTH_dest=SE3(transl(0,0,dist))@MTH_dest
        #print(MTH_act)
        print(MTH_dest)
        #Calcular MTH intermedias, se calculan cada 5mm
        steps=rtb.ctraj(MTH_act,MTH_dest,int(abs(dist)/0.5))
        #print(type(np.array(steps)))
        #Para cada MTH se calcula cinematica inversa y se mueve brazo
        for index in range(len(steps)):
            q=inv_kin(np.array(steps[index]))
            for index1 in range(len(motorID)):
                moveart('', motorID[index1], 'Goal_Position', 180*q[index1]/np.pi, 0)
        #Se actualiza MTH actual
        MTH_act=MTH_dest