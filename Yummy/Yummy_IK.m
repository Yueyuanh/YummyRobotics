%%
% 文件名: Yummy_IK.m
% 作者: 岳远浩
% 版本: v0.01
% 日期: 2025-06-04

% 描述: 
%   机械臂逆运动学解算
%   机械臂逆运动学分为封闭解法和数值解法，封闭解法通过解析方式求封闭解，分为代数法和几何法；数值法通过迭代求解，速度较慢
%   本程序通过几何法解算出前三个关节的角度，通过ZYZ欧拉角变换求解出腕关节的角度
%% 清空
clear;clc;close all;    
%% 加载机器人
Yummy = inc.Yummy_Robot();
import inc.yummy_ik;
%%
p=[0.2;0.1;0.3];
T_rot = trotz(90)*troty(90)*trotz(180);  
R = T_rot(1:3, 1:3); 
T = [R, p; 0 0 0 1];
%%

for i =-0.3:0.01:0.3
    
    T(1,4)=i;
    theta=yummy_ik(T);
    Yummy.plot(theta);
%     Yummy.teach(theta)
end
for i =0.3:-0.01:-0.2
    T(3,4)=i;
    theta=yummy_ik(T);
    Yummy.plot(theta);
%     Yummy.teach(theta)
end
for i =0.1:-0.01:-0.4
    T(2,4)=i;
    theta=yummy_ik(T);
    Yummy.plot(theta);
%     Yummy.teach(theta)
end


% Yummy.teach(init_angles)
