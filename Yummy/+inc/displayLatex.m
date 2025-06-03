function displayLatex(expr,fontSize)
% DISPLAYLATEX  ��ʾ���ű��ʽ��LaTeX��ȾЧ��
%   ���룺
%      expr: ���ű��ʽ��syms����ı�������ű��ʽ��
%   ʾ����
%      syms x;
%      displayLatex(x^2 + 1/x);
    if nargin < 2
        fontSize = 16; % Ĭ��ֵ
    end
    
    % �����޹�������ͼ�δ���
    fig = figure('MenuBar', 'none', 'ToolBar', 'none', 'Name', 'LaTeX Display');
    axis off;
    
    % ��ȾLaTeX��ʽ
    text('Position', [0.0, 0.5], ...
         'Interpreter', 'latex', ...
         'String', ['$$ ' latex(expr) ' $$'], ...
         'FontSize', fontSize);
end