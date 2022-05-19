clear;clf;clc;
%% parametros DH
l=[13.37 10.66 10.66 10.93];
alpha=[pi/2 0 0 0];
a=[0 l(2) l(3) l(4)];
d=[l(1) 0 0 0];
offset=[0 pi/2 0 0];
%% Creacion de eslabones
L(1) = Link('revolute','alpha',alpha(1),'a',a(1),'d',d(1),'offset',offset(1),'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',alpha(2),'a',a(2),'d',d(2),'offset',offset(2),'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',alpha(3),'a',a(3),'d',d(3),'offset',offset(3),'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',alpha(4),'a',a(4),'d',d(4),'offset',offset(4),'qlim',[-3*pi/4 3*pi/4]);
% Definicion de robot
PX=SerialLink(L);
PX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
%% Representacion del robot
ws=[-(l(2)+l(3)+l(4))-1 (l(2)+l(3)+l(4))+1 -(l(2)+l(3)+l(4))-1 (l(2)+l(3)+l(4))+1 -1 (l(1)+l(2)+l(3)+l(4))+2];
figure(1);
q=[0 0 0 0];
PX.plot(deg2rad(q),'workspace',ws);
view(3);
%%
q=[-20 20 -20 20];
PX.plot(deg2rad(q),'workspace',ws);
%%
q=[30 -30 30 -30];
PX.plot(deg2rad(q),'workspace',ws);
%%
q=[-90 15 -55 17];
PX.plot(deg2rad(q),'workspace',ws);
%%
q=[-90 45 -55 45];
PX.plot(deg2rad(q),'workspace',ws);