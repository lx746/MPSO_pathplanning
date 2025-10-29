% LocalSearch 局部搜索函数
% 对当前解 x 在每一维做小范围邻域扰动，若扰动后适应度更优则接受
%
% 输入参数：
%   x    ：当前解（路径节点索引向量）
%   Xmax ：每一维的最大取值（即节点最大索引）
%   G    ：环境地图
%
% 输出参数：
%   x    ：局部搜索后的最优解

function x = LocalSearch(x,Xmax,G)
dim = length(x);            % 维度，即路径节点数
fx = fitness(x,G);          % 当前解的适应度
for i = 1:dim
    newx = x;               % 拷贝一份当前解
    newx(i) = randi(Xmax);  % 在第i维随机扰动
    newfx = fitness(newx,G);% 计算新解适应度
    if newfx < fx           % 如果新解更优
        x = newx;           % 更新解
        fx = newfx;         % 更新最优适应度
    end
end
end
