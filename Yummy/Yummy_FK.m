%%
% �ļ���: Yummy_FK.m
% ����: ��Զ��
% �汾: v0.01
% ����: 2025-05-30

% ����: 
%   ��е�����˶�ѧ����
%   �ؽڿռ�->�ѿ����ռ�
%   �ؽڽǶ�->�ռ�����
%% ���
clear;clc;close all;   
% ��ʽ�����
%% ��е��������
Yummy=inc.Yummy_Robot();
import inc.yummy_fk;
%% 
init_angle=[0,pi/2,0,0,0,0];
T06=yummy_fk(init_angle);
disp(T06);

T = Yummy.fkine(init_angle);
disp(T);