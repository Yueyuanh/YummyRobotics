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
import inc.yummy_ik;
%%
p=[0.2;0.1;0.3];
T_rot = trotx(0)*troty(90)*trotz(180);  
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
