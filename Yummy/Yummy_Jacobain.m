%%
% �ļ���: Yummy_Jacobian.m
% ����: ��Զ��
% �汾: v0.01
% ����: 2025-06-04

% ����: 
%   ��е���ſɱȾ���

%%
%% ���
clear;clc;close all;   
% ��ʽ�����
%% ��е��������
Yummy=inc.Yummy_Robot();
import inc.yummy_ik;

p=[0.2;0.1;0.3];
T_rot = trotz(90)*troty(90)*trotz(180);  
R = T_rot(1:3, 1:3); 
T = [R, p; 0 0 0 1];
%%
q = [0, pi/2, 0, 0, 0, 0];     % �ؽڽǶ�
J = Yummy.jacob0(q);           % ��ȡ6��6 Jacobian����base����ϵ��

% λ�ò��֣�
Jv = J(1:3, :);% ���ٶȲ���
Jw = J(4:6, :);% ���ٶȲ���

disp(J)

% ���ٶȵĿɲ���������
subplot(1,2,1)
plot_ellipse( J(1:3,:)*J(1:3,:)' )
xlabel('x');ylabel('y');zlabel('z')

% ���ٶȵĿɲ���������
subplot(1,2,2)
plot_ellipse( J(4:6,:)*J(4:6,:)' )
xlabel('x');ylabel('y');zlabel('z')

% ����Խ��Բ������ʾ��е���ڸ��������˶�����Խ���⡣
% ����Խ���񡱻�ѹ�⡱��˵����ĳЩ�����˶������

% �ɲ����Զ���
Yummy.maniplty(q)

k=cond(J);
disp(k);
%% ����һ��·���µ����������ж�·�������
theta_list = linspace(-0.3, 0.3, 100);
k_list = zeros(size(theta_list));

% ��������ͼ�δ���
figure(2); clf;
subplot(1,2,1); % ��ࣺ��е�۶���
ax1 = gca;

subplot(1,2,2); % �Ҳࣺ��̬����������
ax2 = gca;
h = animatedline('Parent', ax2, 'LineWidth', 2, 'Color', 'b');
xlabel(ax2, 'x');
ylabel(ax2, 'Cond');
title(ax2, 'Condition number in path');
grid on;
xlim(ax2, [min(theta_list), max(theta_list)]);
ylim(ax2, [0, 100]); % �ɸ�����Ҫ��������

for i = 1:length(theta_list)
    % ������ǰĿ��λ��
    T(1,4) = theta_list(i);

    % ��� & �ſɱ�
    q = yummy_ik(T);
    J = Yummy.jacob0(q);
    k = cond(J);
    k_list(i) = k;

    % ���»�е��ͼ��
    subplot(1,2,1); cla(ax1);
    Yummy.plot(q);
    title(ax1, sprintf('Step %d / %d', i, length(theta_list)));

    % ���¶�̬����
    subplot(1,2,2);
    addpoints(h, theta_list(i), k);
    drawnow;
end

figure
plot(theta_list, k_list);
xlabel('x');
ylabel('Condition Number');
title('Condition number vs path');

