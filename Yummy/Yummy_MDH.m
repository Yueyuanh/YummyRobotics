%%
% �ļ���: Yummy_MDH.m
% ����: ��Զ��
% �汾: v0.01
% ����: 2025-05-30

% ����: 
%   ʹ�øĽ�DH���Ի����˽��н�ģ

%% ���
clear;clc;close all;   
%%
% ���ػ�����ģ��

Yummy=inc.Yummy_Robot();

%% 
% ���ӻ�������
figure;
% ��ʼλ��
init_angles=[0 pi/2 0 0 0 0];
Yummy.plot([0 pi/2 0 0 0 0]);

% Yummy.teach(init_angles)

title('Yummy Robot(MDH)');
xlabel('X'); ylabel('Y'); zlabel('Z');