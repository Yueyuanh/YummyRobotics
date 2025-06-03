% Yummy_Robot.m
function Yummy_Robot = Yummy_Robot()
    % ����: ��Զ��
    % �汾: v0.01
    % ����: 2025-05-30
   
    
    % ���˶��壬���ݹؽ�����RP,��ת�ؽڴ����ʼ���Ƕ�
    % theta:�ؽڽǶ�(rad)  d:����ƫ��(m)  a:���˳���(m)  alpha:����Ť��(rad)

    L(1) = Link('d', 0, 'a', 0,   'alpha', 0, 'modified');
    L(2) = Link('d', 0, 'a', 0,    'alpha', pi/2, 'modified');
    L(3) = Link('d', 0, 'a', 0.3,  'alpha', 0, 'modified');
    L(4) = Link('d', 0.27, 'a', 0.096, 'alpha',pi/2, 'modified');
    L(5) = Link('d', 0, 'a', 0,  'alpha', -pi/2, 'modified');
    L(6) = Link('d', 0.107, 'a', 0, 'alpha', pi/2, 'modified');
    
    % ���ùؽ�����
    L(1).qlim = [-180 180]*pi/180;
    L(2).qlim = [0 180]*pi/180;
    L(3).qlim = [-90 90]*pi/180;
    L(4).qlim = [-90 90]*pi/180;
    L(5).qlim = [-90 90]*pi/180;
    L(6).qlim = [-180 180]*pi/180;
    
    % ���������˶���
    Yummy_Robot = SerialLink(L, 'name', 'Yummy Robot');
    
    % ��ʾ�����˲���
    Yummy_Robot.display()

end