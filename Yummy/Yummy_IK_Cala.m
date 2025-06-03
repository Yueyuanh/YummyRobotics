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
syms a1 a2 a3 d1 d2 d3 d4 real
syms alpha1 alpha2 alpha3 real
syms theta1 theta2 theta3 real
syms f1 f2 f3 real
% 建立三角函数简写关系
c1 = cos(theta1); s1 = sin(theta1);
c2 = cos(theta2); s2 = sin(theta2);
c3 = cos(theta3); s3 = sin(theta3);

% 定义f1, f2, f3 (修正后的正确表达式)
% f1 = a3*c3 + d4*sin(alpha3)*s3 + a2;
% f2 = a3*cos(alpha2)*s3 - d4*sin(alpha3)*cos(alpha2)*c3 - d4*sin(alpha2)*cos(alpha3) - d3*sin(alpha2);
% f3 = a3*sin(alpha2)*s3 - d4*sin(alpha3)*sin(alpha2)*c3 + d4*cos(alpha2)*cos(alpha3) + d3*cos(alpha2);

% 定义g1, g2, g3 (修正后的正确表达式)
g1 = c2*f1 - s2*f2 + a1;
g2 = s2*cos(alpha1)*f1 + c2*cos(alpha1)*f2 - sin(alpha1)*f3 - d2*sin(alpha1);
g3 = s2*sin(alpha1)*f1 + c2*sin(alpha1)*f2 - cos(alpha1)*f3 - d2*cos(alpha1);

% 计算r = g1? + g2? + g3?
r = g1^2 + g2^2 + g3^2;

% 展开并简化表达式
r_expanded = expand(r);
r_simplified = simplify(r_expanded);

% 显示结果
disp('原始r表达式:');
pretty(r)

% disp('展开后的r表达式:');
% pretty(r_expanded)
% 
disp('简化后的r表达式:');
pretty(r_simplified)

% displayLatex(r_simplified,10);

% 在原有代码后添加LaTeX输出
% latex_r = latex(r_simplified);
% disp('LaTeX代码:');
% disp(latex_r);

% 
% % 可选：按特定变量分组显示
disp('按a1,a2,a3分组的表达式:');
r_collected = collect(r_simplified, [f1 f2 f3]);
pretty(r_collected)
displayLatex(r_collected)