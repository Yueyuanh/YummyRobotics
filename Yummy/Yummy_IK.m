%%
% 文件名: Yummy_IK.m
% 作者: 岳远浩
% 版本: v0.01
% 日期: 2025-05-31

% 描述: 
%   机械臂逆运动学解算
%   机械臂逆运动学分为封闭解法和数值解法，封闭解法通过解析方式求封闭解，分为代数法和几何法；数值法通过迭代求解，速度较慢
%% 清空
clear;clc;close all;    
%% 加载机器人
Yummy = inc.Yummy_Robot();

%%
% 定义目标位姿（齐次变换矩阵）
T = transl(0.5, 0.1, 0.2) * trotx(pi);

disp(T)
% 计算逆运动学解
q = Yummy.ikine(T);
disp(q)
% Yummy.teach(init_angles)
% Yummy.teach(q)