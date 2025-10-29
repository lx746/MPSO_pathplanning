function h = drawPath(path, col)
% drawPath 画路径函数
% 在当前坐标系上按照给定的路径节点和颜色绘制路径
%
% 输入参数：
%   path ：路径点序列（每行为一个坐标点 [行 列]）
%   col  ：路径线条颜色（如 'b', 'r', [0 0.5 1] 等）
%
% 输出参数：
%   h    ：最后一段路径的句柄

    hold on
    % L为路径点总个数
    L = size(path, 1);

    % 绘制起点（红色圆圈）
    Sx = path(1, 1) - 0.5;     % 起点行坐标（减0.5使其居中对齐格子）
    Sy = path(1, 2) - 0.5;     % 起点列坐标
    plot(Sy, Sx, 'ro', 'MarkerSize', 4, 'LineWidth', 4);

    % 绘制路径线条
    for i = 1:L-1
        % 连线每相邻两个点
        h = plot([path(i, 2) path(i+1, 2)] - 0.5, ...
                 [path(i, 1) path(i+1, 1)] - 0.5, ...
                 'Color', col, 'LineWidth', 2);
        hold on
    end

    % 绘制终点（红色方块）
    Ex = path(end, 1) - 0.5;   % 终点行坐标
    Ey = path(end, 2) - 0.5;   % 终点列坐标
    % Ex = 49.5;
    % Ey = 49.5;
    plot(Ey, Ex, 'rs', 'MarkerSize', 4, 'LineWidth', 4);
end