function T_inv = Tinv(T)
    % 提取旋转部分和平移部分
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    % 计算逆矩阵
    T_inv = [R', -R'*p; 
             0, 0, 0, 1];
end