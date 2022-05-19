#!/usr/bin/env python
from operator import le
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand
import termios, sys, tty
import math

TERMIOS=termios

artID=6
HomeWaist=0
HomeShoulder=0
HomeElbow=0
HomeWrist=0

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

def getkey():
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    x = 0
    x=sys.stdin.read(1)[0]
    return x

def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", jointState, callback)
    rospy.spin()
    
def callback(data):
    rospy.loginfo(data.position)
    currentAng[0]=data.position[0]
    currentAng[1]=data.position[1]
    currentAng[2]=data.position[2]
    currentAng[3]=data.position[3]
    print(data.position)
    

def nextArt(ID):
    if ID==9:
        ID=6
    else:
        ID+=1
    return ID

def prevArt(ID):
    if ID==6:
        ID=9
    else:
        ID-=1
    return ID

currentAng=[]

i=1
if __name__ == '__main__':
    moveart('', 6, 'Torque_Limit', 3*1024/4-1, 0)
    moveart('', 7, 'Torque_Limit', 2*1024/3-1, 0)
    moveart('', 8, 'Torque_Limit', 1024/2-1, 0)
    moveart('', 9, 'Torque_Limit', 1024/3-1, 0)
    while i==1:
        Key=getkey()
        if Key==chr(72) or Key==chr(104): #Tecla H mueve articulaciones a HOME
                moveart('', 6, 'Goal_Position', HomeWaist, 0.5)
                moveart('', 7, 'Goal_Position', HomeShoulder, 0.5)
                moveart('', 8, 'Goal_Position', HomeElbow, 0.5)
                moveart('', 9, 'Goal_Position', HomeWrist, 0.5)
        elif Key==chr(87) or Key==chr(119): #Tecla W mueve entre articulacion
            artID=nextArt(artID)
        elif Key==chr(83) or Key==chr(115): #Tecla S mueve entre articulacion
            artID=prevArt(artID)
        elif Key==chr(65) or Key==chr(97): #Tecla A mueve articulacion a izq
            try:
                moveart('', artID, 'Goal_Position', -90, 0.1)
            except rospy.ROSInterruptException:
                pass
        elif Key==chr(68) or Key==chr(100): #Tecla D mueve articulacion a der
            try:
                moveart('', artID, 'Goal_Position', 90, 0.1)
            except rospy.ROSInterruptException:
                pass
        elif Key==chr(82) or Key==chr(114): #Tecla R mueve articulacion a home
            if artID==6:
                try:
                    moveart('', 6, 'Goal_Position', HomeWaist, 0)
                except rospy.ROSInterruptException:
                    pass
            elif artID==7:
                try:
                    moveart('', 7, 'Goal_Position', HomeShoulder, 0)
                except rospy.ROSInterruptException:
                    pass
            elif artID==8:
                try:
                    moveart('', 8, 'Goal_Position', HomeElbow, 0)
                except rospy.ROSInterruptException:
                    pass
            elif artID==9:
                try:
                    moveart('', 9, 'Goal_Position', HomeWrist, 0)
                except rospy.ROSInterruptException:
                    pass
        elif Key==chr(27): #ESC
            sys.exit()
