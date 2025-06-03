%%
% 文件名: Yummy_MDH.m
% 作者: 岳远浩
% 版本: v0.01
% 日期: 2025-05-30

% 描述: 
%   使用改进DH法对机器人进行建模

%% 清空
clear;clc;close all;   
%%
% 加载机器人模型

Yummy=inc.Yummy_Robot();

%% 
% 可视化机器人
figure;
% 初始位置
init_angles=[0 pi/2 0 0 0 0];
Yummy.plot([0 pi/2 0 0 0 0]);

% Yummy.teach(init_angles)

title('Yummy Robot(MDH)');
xlabel('X'); ylabel('Y'); zlabel('Z');