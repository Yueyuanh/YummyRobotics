%%
% �ļ���: Yummy_Workspace.m
% ����: ��Զ��
% �汾: v0.01
% ����: 2025-05-30

% ����: 
%   ��е�۹����ռ���ӻ�
%   ��ȡ������ģ�ͺ͸��ؽڽǶȷ�Χ->���ؿ���������Ը����ؽھ��Ȳ������ɴ�->���ؽڿռ䣩����������Ƕȴ������˶�ѧ���������ѿ�������

%% ���
clear;clc;close all;    
%% ���ػ�����
Yummy = inc.Yummy_Robot();

%% �����ռ�
% �����ռ��Ϊ���ɴ﹤���ռ�������ռ�
% ����˼�壬�ɴ﹤���ռ��ǻ�е���ܹ���һ�����ϵķ��򵽴��λ�õļ���
% �����ռ��������ⷽ�򵽴��Ŀ���ļ��ϣ������ռ��ǿɴ﹤���ռ���Ӽ�


%% ���ؿ������
N = 10000;
% �����˹ؽ���
num_joints=Yummy.n;
theta_rand = zeros(N, num_joints);


for i = 1:N
    for j = 1:num_joints
        min=Yummy.links(j).qlim(1);
        max=Yummy.links(j).qlim(2);
        % �����ֵ��Сֵ����Ȳ���
        theta_rand(i,j)=min+rand()*(max-min);
    end
end

%% ���ݲ��������ռ�λ�ã����˶�ѧ��
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


%% ��̬���ӻ�
figure(1); 
for i = 1:100:N
    theta = theta_rand(i,:);
    Yummy.plot(theta);
end
Yummy.plot(init_angles);
hold off;
