%%
% �ļ���: Yummy_IK_Cala.m
% ����: ��Զ��
% �汾: v0.01
% ����: 2025-05-31

% ����: 

%% ���
clear;clc;close all;
%% 
import inc.displayLatex

% �������з��ű���
syms a1 a2 a3 d1 d2 d3 d4 real
syms alpha1 alpha2 alpha3 real
syms theta1 theta2 theta3 real
syms f1 f2 f3 real
% �������Ǻ�����д��ϵ
c1 = cos(theta1); s1 = sin(theta1);
c2 = cos(theta2); s2 = sin(theta2);
c3 = cos(theta3); s3 = sin(theta3);

% ����f1, f2, f3 (���������ȷ���ʽ)
% f1 = a3*c3 + d4*sin(alpha3)*s3 + a2;
% f2 = a3*cos(alpha2)*s3 - d4*sin(alpha3)*cos(alpha2)*c3 - d4*sin(alpha2)*cos(alpha3) - d3*sin(alpha2);
% f3 = a3*sin(alpha2)*s3 - d4*sin(alpha3)*sin(alpha2)*c3 + d4*cos(alpha2)*cos(alpha3) + d3*cos(alpha2);

% ����g1, g2, g3 (���������ȷ���ʽ)
g1 = c2*f1 - s2*f2 + a1;
g2 = s2*cos(alpha1)*f1 + c2*cos(alpha1)*f2 - sin(alpha1)*f3 - d2*sin(alpha1);
g3 = s2*sin(alpha1)*f1 + c2*sin(alpha1)*f2 - cos(alpha1)*f3 - d2*cos(alpha1);

% ����r = g1? + g2? + g3?
r = g1^2 + g2^2 + g3^2;

% չ�����򻯱��ʽ
r_expanded = expand(r);
r_simplified = simplify(r_expanded);

% ��ʾ���
disp('ԭʼr���ʽ:');
pretty(r)

% disp('չ�����r���ʽ:');
% pretty(r_expanded)
% 
disp('�򻯺��r���ʽ:');
pretty(r_simplified)

% displayLatex(r_simplified,10);

% ��ԭ�д�������LaTeX���
% latex_r = latex(r_simplified);
% disp('LaTeX����:');
% disp(latex_r);

% 
% % ��ѡ�����ض�����������ʾ
disp('��a1,a2,a3����ı��ʽ:');
r_collected = collect(r_simplified, [f1 f2 f3]);
pretty(r_collected)
displayLatex(r_collected)