% GenerateSmoothPath 路径平滑函数
% 通过删除路径中多余的中间节点，使路径更平滑（只要首尾间无障碍即可直连）
%
% 输入参数：
%   path ：路径点序列，每行为一个坐标点 [行 列]
%   G    ：环境地图（0为可通行，1为障碍）
%
% 输出参数：
%   path1：平滑后的路径点序列

function path1 = GenerateSmoothPath(path, G)
    path1 = path;                 % 复制一份原始路径
    long = size(path, 1);         % 路径点数量
    i = 1;                        % 当前检查的路径点索引
    while i ~= long - 2           % 遍历除最后两个点外的所有点
        a1 = path1(i, 1);         % 当前点的行坐标
        b1 = path1(i, 2);         % 当前点的列坐标

        a3 = path1(i + 2, 1);     % 后两个点的行坐标
        b3 = path1(i + 2, 2);     % 后两个点的列坐标

        if a1 < a3
            % 如果从a1到a3之间、b1到b3之间的区域无障碍
            if all(G(a1:a3, b1:b3) == 0)
                path1(i + 1, :) = [];    % 删除中间点
                i = i - 1;               % 指针前移，重新检查
            end
        else
            % 如果从a3到a1之间、b1到b3之间的区域无障碍
            if all(G(a3:a1, b1:b3) == 0)
                path1(i + 1, :) = [];    % 删除中间点
                i = i - 1;               % 指针前移，重新检查
            end
        end
        i = i + 1;                       % 检查下一个点
        long = size(path1, 1);           % 路径长度随时更新
    end
end

