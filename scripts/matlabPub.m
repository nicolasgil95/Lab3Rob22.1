clear; clc;
rosinit;
%%
%Pub = rospublisher('','dynamixel_workbench_msgs/DynamixelCommandRequest');%Creacion del publicador
%Msg = rosmessage(velPub); %Creacion del mensaje
%Msg.Linear.X = 1;
%send(Pub,Msg);

PxCommSrv= rossvcclient('/dynamixel_workbench/dynamixel_command'); %creacion cliente de servicio
CommandMsg= rosmessage(PxCommSrv);  %Creacion del mensaje 
CommandMsg.Id=8 %def def ID motor a mover
CommandMsg.AddrName='Goal_Position' %def nombre del registro a usar
CommandMsg.Value = 715 %|ef angulo, 715b=53.5254°
call(PxCommSrv,CommandMsg)
%%
rosshutdown;