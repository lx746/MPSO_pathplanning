% Bounds 简单边界处理函数
% 保证粒子位置在给定的上下界范围内，超界则随机重置为合法值
%
% 输入参数：
%   s    ：粒子当前位置（向量）
%   Xmin ：允许的最小值
%   Xmax ：允许的最大值
%
% 输出参数：
%   s    ：修正后的粒子位置

function s = Bounds(s, Xmin, Xmax)

% 找到超上界的分量，将其随机重置为 [1, Xmax] 区间内的整数
index = find(s > Xmax);
s(index) = randi(Xmax);

% 找到超下界的分量，同样重置为 [1, Xmax] 区间内的整数（注意不是Xmin）
index = find(s < Xmin);
s(index) = randi(Xmax);

% 最后对所有分量取整，保证为整数格点
s = round(s);

end