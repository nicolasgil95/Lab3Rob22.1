#!/usr/bin/env python
from turtle import home
import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
import numpy as np

l = np.array([14.5, 10.7, 10.7, 9])
q=[0 0 0 0 0]
qlims = np.array([[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4]])

PX = rtb.DHRobot(
    [rtb.RevoluteDH(alpha=np.pi/2, d=l[0], qlim=qlims[0,:]),
    rtb.RevoluteDH(a=l[1], offset=np.pi/2, qlim=qlims[0,:]),
    rtb.RevoluteDH(a=l[2], qlim=qlims[0,:]),
    rtb.RevoluteDH(qlim=qlims[0,:])],
    name="PhantomX")
PX.tool = transl(l[3],0,0).dot(troty(np.pi/2).dot(trotz(-np.pi/2)))

if __name__ == '__main__':
    llevar a home
    llevar a pos1
    llevar a pos2
    llevar a pos3
    llevar a pos4
    llevar a pos3
    llevar a pos5
    llevar a pos6
