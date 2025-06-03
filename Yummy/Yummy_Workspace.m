%%
% 文件名: Yummy_Workspace.m
% 作者: 岳远浩
% 版本: v0.01
% 日期: 2025-05-30

% 描述: 
%   机械臂工作空间可视化
%   获取机器人模型和各关节角度范围->蒙特卡洛采样：对各个关节均匀采样若干次->（关节空间）采样的随机角度代入正运动学方程中求解笛卡尔坐标

%% 清空
clear;clc;close all;    
%% 加载机器人
Yummy = inc.Yummy_Robot();

%% 工作空间
% 工作空间分为，可达工作空间和灵活工作空间
% 顾名思义，可达工作空间是机械臂能够以一种以上的方向到达的位置的集合
% 灵活工作空间则是任意方向到达的目标点的集合，灵活工作空间是可达工作空间的子集


%% 蒙特卡洛采样
N = 10000;
% 机器人关节数
num_joints=Yummy.n;
theta_rand = zeros(N, num_joints);


for i = 1:N
    for j = 1:num_joints
        min=Yummy.links(j).qlim(1);
        max=Yummy.links(j).qlim(2);
        % 在最大值最小值间均匀采样
        theta_rand(i,j)=min+rand()*(max-min);
    end
end

%% 根据采样点计算空间位置（正运动学）
points=zeros(N,3);

for i = 1:N
    theta=theta_rand(i,:);
    workspace=Yummy.fkine(theta);
    points(i,:)=workspace.t';
end

figure(1); 
scatter3(points(:,1), points(:,2), points(:,3), 2, 'b', 'filled');
init_angles=[0 pi/2 0 0 0 0];
Yummy.teach(init_angles)


%% 动态可视化
figure(1); 
for i = 1:100:N
    theta = theta_rand(i,:);
    Yummy.plot(theta);
end
Yummy.plot(init_angles);
hold off;
