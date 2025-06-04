%%
% 文件名: yummy_fk.m
% 作者: 岳远浩
% 版本: v0.01
% 日期: 2025-06-4

% 描述: 正运动学解算
%%

function T06=yummy_fk(theta)

    theta1 = theta(1);
    theta2 = theta(2);
    theta3 = theta(3);
    theta4 = theta(4);
    theta5 = theta(5);
    theta6 = theta(6);
    
    c1=cos(theta1);c2=cos(theta2);c3=cos(theta3); 
    c4=cos(theta4);c5=cos(theta5);c6=cos(theta6); 
    s1=sin(theta1);s2=sin(theta2);s3=sin(theta3);
    s4=sin(theta4);s5=sin(theta5);s6=sin(theta6);

    T01=[c1,-s1,0,0;
         s1, c1,0,0;
          0,  0,1,0;
          0,  0,0,1];
    T12=[c2,-s2, 0,0;
         0 ,  0,-1,0;
         s2, c2, 0,0;
          0,  0, 0,1];
    T23=[c3,-s3,0,0.3;
         s3, c3,0,  0;
          0,  0,1,  0;
          0,  0,0,  1];
    T34=[c4,-s4, 0,0.096;
          0,  0,-1,-0.27;
         s4, c4, 0,    0;
          0,  0, 0,    1];
    T45=[ c5,-s5, 0,0;
           0,  0, 1,0;
         -s5,-c5, 0,0;
          0,  0,  0,1];
    T56=[c6,-s6, 0,0;
          0,  0,-1,-0.107;
         s6, c6, 0,0;
          0,  0, 0,1];

    T06=T01*T12*T23*T34*T45*T56;
end