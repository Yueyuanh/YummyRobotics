%%
% �ļ���: Yummy_IK.m
% ����: ��Զ��
% �汾: v0.01
% ����: 2025-05-31

% ����: 
%   ��е�����˶�ѧ����
%   ��е�����˶�ѧ��Ϊ��սⷨ����ֵ�ⷨ����սⷨͨ��������ʽ���ս⣬��Ϊ�������ͼ��η�����ֵ��ͨ��������⣬�ٶȽ���
%% ���
clear;clc;close all;    
%% ���ػ�����
Yummy = inc.Yummy_Robot();

%%
% ����Ŀ��λ�ˣ���α任����
T = transl(0.5, 0.1, 0.2) * trotx(pi);

disp(T)
% �������˶�ѧ��
q = Yummy.ikine(T);
disp(q)
% Yummy.teach(init_angles)
% Yummy.teach(q)