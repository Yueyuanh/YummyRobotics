function displayLatex(expr,fontSize)
% DISPLAYLATEX  显示符号表达式的LaTeX渲染效果
%   输入：
%      expr: 符号表达式（syms定义的变量或符号表达式）
%   示例：
%      syms x;
%      displayLatex(x^2 + 1/x);
    if nargin < 2
        fontSize = 16; % 默认值
    end
    
    % 创建无工具栏的图形窗口
    fig = figure('MenuBar', 'none', 'ToolBar', 'none', 'Name', 'LaTeX Display');
    axis off;
    
    % 渲染LaTeX公式
    text('Position', [0.0, 0.5], ...
         'Interpreter', 'latex', ...
         'String', ['$$ ' latex(expr) ' $$'], ...
         'FontSize', fontSize);
end