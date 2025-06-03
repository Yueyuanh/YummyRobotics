% Yummy_Robot.m
function Yummy_Robot = Yummy_Robot()
    % 作者: 岳远浩
    % 版本: v0.01
    % 日期: 2025-05-30
   
    
    % 连杆定义，根据关节类型RP,旋转关节传入初始化角度
    % theta:关节角度(rad)  d:连杆偏距(m)  a:连杆长度(m)  alpha:连杆扭角(rad)

    L(1) = Link('d', 0, 'a', 0,   'alpha', 0, 'modified');
    L(2) = Link('d', 0, 'a', 0,    'alpha', pi/2, 'modified');
    L(3) = Link('d', 0, 'a', 0.3,  'alpha', 0, 'modified');
    L(4) = Link('d', 0.27, 'a', 0.096, 'alpha',pi/2, 'modified');
    L(5) = Link('d', 0, 'a', 0,  'alpha', -pi/2, 'modified');
    L(6) = Link('d', 0.107, 'a', 0, 'alpha', pi/2, 'modified');
    
    % 设置关节限制
    L(1).qlim = [-180 180]*pi/180;
    L(2).qlim = [0 180]*pi/180;
    L(3).qlim = [-90 90]*pi/180;
    L(4).qlim = [-90 90]*pi/180;
    L(5).qlim = [-90 90]*pi/180;
    L(6).qlim = [-180 180]*pi/180;
    
    % 创建机器人对象
    Yummy_Robot = SerialLink(L, 'name', 'Yummy Robot');
    
    % 显示机器人参数
    Yummy_Robot.display()

end