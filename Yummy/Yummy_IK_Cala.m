%%
% 文件名: Yummy_IK_Cala.m
% 作者: 岳远浩
% 版本: v0.01
% 日期: 2025-05-31

% 描述: 

%% 清空
clear;clc;close all;
%% 
import inc.displayLatex

% 定义所有符号变量
syms a2 a3 d4 real
syms alpha1 alpha2 alpha3 alpha4 alpha5 real
syms theta1 theta2 theta3 theta4 theta5 theta6 real
syms x y z real

a2=0.3;a3=0.096;
d4=0.27;
alpha1=pi/2;alpha3=pi/2;alpha4=-pi/2;alpha5=pi/2;

l1=a2;
l2=sqrt((a3^2+d4^2));
alpha=atan2(d4,a3);
Yummy=inc.Yummy_Robot();
import inc.Tinv;

%% 笛卡尔空间坐标
% x=0.3;
% y=-0.4;
% z=-0.2;

p=[0.2;0;0.3];
T_rot = trotx(0)*troty(90)*trotz(180);  
R = T_rot(1:3, 1:3); 
T = [R, p; 0 0 0 1];
%%
x=T(1,4);
y=T(2,4);
z=T(3,4);

%% theta1
theta1=atan2(y,x);

x=sqrt(x^2+y^2);
% disp(x)
%% theta2,theta3


cl2=(x^2+z^2-l1^2-l2^2)/(2*l1*l2);
% disp(cl2)
theta33=acos(cl2);
disp(theta33)
theta3=alpha-acos(cl2);


cphi=(l2^2-(x^2+z^2)-l1^2)/(-2*l1*sqrt(x^2+z^2));
phi=acos(cphi);
% disp("phi:")
% disp(phi)
if theta33>=0
    theta2=atan2(z,x)+phi;
else
    theta2=atan2(z,x)-phi;
end
disp(cl2)
if cl2>1||cl2<-1
    theta1=0;theta2=pi/2;theta3=0;
    disp("无解")
end
%% 后三角
% 计算腕关节的Tw矩阵
% Tw*(T)=Tset
% T=Tw^(-1)*Tset

%% 腕关节正运动学变换
c1=cos(theta1);c2=cos(theta2);c3=cos(theta3); 
s1=sin(theta1);s2=sin(theta2);s3=sin(theta3);
q = [0, pi/2, 0, 0, 0, 0];

T01=[c1,-s1,0,0;
     s1, c1,0,0;
      0,  0,1,0;
      0,  0,0,1];


T12=[c2,-s2, 0,0;
     0 ,  0,-1,0;
     s2, c2, 0,0;
      0,  0, 0,1];

T23=[c3,-s3,0,0.3;
     s3, c3,0,  0;
      0,  0,1,  0;
      0,  0,0,  1];

 T34=[1,0,0,0.096;
      0,0,-1,-0.27;
      0,1,0,0;
      0,0,0,1];

T04=T01*T12*T23*T34;
disp(T04)

%% 求出相对于{4}的旋转矩阵
T1=Tinv(T04)*T;
disp("T04")
disp(T04)

% q = [0, pi/2, 0, 0, 0, 0];
% TT04 = Yummy.A(1, q) * Yummy.A(2, q) * Yummy.A(3, q) * Yummy.A(4, q);
% disp("TT04")
% disp(TT04)
%%
theta5=atan2(sqrt(T1(3,1)^2+T1(3,2)^2),T1(3,3));
s5=sin(theta5);
disp(theta5)

theta4=atan2(T1(2,3)/s5,T1(1,3)/s5);
theta6=atan2(T1(3,2)/s5,-T1(3,1)/s5);

if theta5==0
    theta4=0;
    theta6=atan2(-T1(1,2),T1(1,1));
end

if theta5==pi
    theta4=0;
    theta6=atan2(T1(1,2),-T1(1,1));
end



%% 验证
theta=[theta1,theta2,theta3,theta4,theta5,theta6];
disp(theta)

Yummy.teach(theta)
disp(theta)
if cl2>1||cl2<-1
    disp("无解")
end
% disp(T04)
q=[theta1,theta2,theta3,0,0,0];
Tq = Yummy.fkine(q);
% disp(Tq);

