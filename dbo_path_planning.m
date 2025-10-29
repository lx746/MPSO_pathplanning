% 基于蜣螂算法的移动机器人路径规划
% 作者：Copilot
clc; clear; close all;

%% 参数设置
map_size = [50, 50];           % 地图尺寸
obstacle_rate = 0.2;           % 障碍物比例
start_point = [2, 2];
goal_point = [49, 49];
pop_size = 30;                 % 蜣螂个体数量
max_iter = 100;                % 最大迭代次数
path_len = 20;                 % 路径点数

%% 随机生成地图
map = zeros(map_size);
obstacles = rand(map_size) < obstacle_rate;
map(obstacles) = 1;
map(start_point(1), start_point(2)) = 0;
map(goal_point(1), goal_point(2)) = 0;

%% 蜣螂个体初始化
pop = cell(pop_size,1);
fitness = zeros(pop_size,1);

for i = 1:pop_size
    path = [linspace(start_point(1), goal_point(1), path_len)', ...
            linspace(start_point(2), goal_point(2), path_len)'];
    % 添加随机扰动
    path = path + randn(size(path));
    pop{i} = round(path);
    fitness(i) = path_fitness(pop{i}, map, goal_point);
end

%% 主循环
for iter = 1:max_iter
    % 按适应度排序
    [fitness, idx] = sort(fitness);
    pop = pop(idx);
    best_path = pop{1};
    best_fit = fitness(1);

    for i = 1:pop_size
        % 更新位置（蜣螂算法核心思想：向最优个体靠近 + 扰动）
        new_path = pop{i} + randn(size(pop{i})) + 0.2*(best_path - pop{i});
        new_path = round(new_path);
        % 限制在地图范围内
        % new_path(new_path<1) = 1;
        % new_path(:,1)(new_path(:,1)>map_size(1)) = map_size(1);
        % new_path(:,2)(new_path(:,2)>map_size(2)) = map_size(2);
        % 限制在地图范围内
        new_path(new_path<1) = 1;
        idx1 = new_path(:,1) > map_size(1);
        new_path(idx1,1) = map_size(1);
        idx2 = new_path(:,2) > map_size(2);
        new_path(idx2,2) = map_size(2);
        % 保证首尾为起点和终点
        new_path(1,:) = start_point;
        new_path(end,:) = goal_point;
        % 计算适应度
        new_fit = path_fitness(new_path, map, goal_point);
        % 选择
        if new_fit < fitness(i)
            pop{i} = new_path;
            fitness(i) = new_fit;
        end
    end
    % 显示进度
    if mod(iter,10)==0
        fprintf('迭代次数：%d，最佳路径代价：%.2f\n', iter, best_fit);
    end
end

%% 绘制结果
figure; imagesc(map'); axis xy; hold on;
colormap(gray);
plot(best_path(:,1), best_path(:,2), 'r-o', 'LineWidth',2);
plot(start_point(1), start_point(2), 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
plot(goal_point(1), goal_point(2), 'bo', 'MarkerSize',10, 'MarkerFaceColor','b');
title('基于蜣螂算法的路径规划');
legend('路径', '起点', '终点');
hold off;

%% 适应度函数
function f = path_fitness(path, map, goal)
    f = 0;
    for i = 1:size(path,1)-1
        % 路径长度
        f = f + norm(path(i+1,:)-path(i,:));
        % 碰撞检测
        if map(path(i,1), path(i,2))==1
            f = f + 100; % 碰撞惩罚
        end
    end
    % 终点偏移
    f = f + norm(path(end,:)-goal)*10;
end