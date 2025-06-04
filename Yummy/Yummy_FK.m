%%
% 文件名: Yummy_FK.m
% 作者: 岳远浩
% 版本: v0.01
% 日期: 2025-05-30

% 描述: 
%   机械臂正运动学解算
%   关节空间->笛卡尔空间
%   关节角度->空间坐标
%% 清空
clear;clc;close all;   
% 格式化输出
%% 机械臂正解算
Yummy=inc.Yummy_Robot();
import inc.yummy_fk;
%% 
init_angle=[0,pi/2,0,0,0,0];
T06=yummy_fk(init_angle);
disp(T06);

T = Yummy.fkine(init_angle);
disp(T);