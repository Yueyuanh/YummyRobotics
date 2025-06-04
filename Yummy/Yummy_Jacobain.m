%%
% 文件名: Yummy_Jacobian.m
% 作者: 岳远浩
% 版本: v0.01
% 日期: 2025-06-04

% 描述: 
%   机械臂雅可比矩阵

%%
%% 清空
clear;clc;close all;   
% 格式化输出
%% 机械臂正解算
Yummy=inc.Yummy_Robot();
import inc.yummy_ik;

p=[0.2;0.1;0.3];
T_rot = trotz(90)*troty(90)*trotz(180);  
R = T_rot(1:3, 1:3); 
T = [R, p; 0 0 0 1];
%%
q = [0, pi/2, 0, 0, 0, 0];     % 关节角度
J = Yummy.jacob0(q);           % 获取6×6 Jacobian，在base坐标系下

% 位置部分：
Jv = J(1:3, :);% 线速度部分
Jw = J(4:6, :);% 角速度部分

disp(J)

% 线速度的可操作性椭球
subplot(1,2,1)
plot_ellipse( J(1:3,:)*J(1:3,:)' )
xlabel('x');ylabel('y');zlabel('z')

% 角速度的可操作性椭球
subplot(1,2,2)
plot_ellipse( J(4:6,:)*J(4:6,:)' )
xlabel('x');ylabel('y');zlabel('z')

% 椭球越“圆”，表示机械臂在各方向上运动能力越均衡。
% 椭球越“瘪”或“压扁”，说明在某些方向运动能力差。

% 可操作性度量
Yummy.maniplty(q)

k=cond(J);
disp(k);
%% 计算一条路径下的条件数，判断路径奇异点
theta_list = linspace(-0.3, 0.3, 100);
k_list = zeros(size(theta_list));

% 创建两个图形窗口
figure(2); clf;
subplot(1,2,1); % 左侧：机械臂动画
ax1 = gca;

subplot(1,2,2); % 右侧：动态条件数曲线
ax2 = gca;
h = animatedline('Parent', ax2, 'LineWidth', 2, 'Color', 'b');
xlabel(ax2, 'x');
ylabel(ax2, 'Cond');
title(ax2, 'Condition number in path');
grid on;
xlim(ax2, [min(theta_list), max(theta_list)]);
ylim(ax2, [0, 100]); % 可根据需要调整上限

for i = 1:length(theta_list)
    % 创建当前目标位姿
    T(1,4) = theta_list(i);

    % 逆解 & 雅可比
    q = yummy_ik(T);
    J = Yummy.jacob0(q);
    k = cond(J);
    k_list(i) = k;

    % 更新机械臂图像
    subplot(1,2,1); cla(ax1);
    Yummy.plot(q);
    title(ax1, sprintf('Step %d / %d', i, length(theta_list)));

    % 更新动态曲线
    subplot(1,2,2);
    addpoints(h, theta_list(i), k);
    drawnow;
end

figure
plot(theta_list, k_list);
xlabel('x');
ylabel('Condition Number');
title('Condition number vs path');

