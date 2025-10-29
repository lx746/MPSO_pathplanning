%%
% 主程序：基于粒子群优化的路径规划
clc
clear
close all
% tic

%% 地图初始化
% 构建20x20的栅格地图，0为自由栅格，1为障碍物
% G = [0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0; 
%      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
%      0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0; 
%      0 0 0 0 0 0 0 1 1 0 0 0 0 0 1 1 1 1 0 0; 
%      0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 1 1 1 0 0; 
%      0 1 1 1 0 0 0 1 1 0 0 0 0 1 0 0 0 0 0 0; 
%      0 1 1 1 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0;
%      0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
%      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
%      1 0 0 0 0 0 0 1 1 1 1 0 0 0 1 0 0 0 0 0; 
%      1 1 1 0 0 0 0 1 1 1 1 0 0 0 1 0 0 0 0 0; 
%      1 1 1 0 0 0 0 1 1 1 1 0 0 0 1 0 0 0 0 0; 
%      1 1 1 0 0 0 0 1 1 1 1 0 0 0 0 0 1 1 1 0; 
%      1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 0; 
%      1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 0; 
%      1 1 0 0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 0 0; 
%      0 0 0 0 0 0 1 1 0 1 1 1 0 0 0 0 0 1 1 0; 
%      0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0; 
%      0 0 0 1 1 0 0 0 0 0 1 1 0 0 1 0 0 0 0 0; 
%      0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0];
map_size = 20; % 可修改为 20、30、50

% map_size = 50;
%  % rng(1);
% G = zeros(map_size, map_size);
% % num_obstacles = round(0.2 * map_size * map_size);
% obstacle_idx = randperm(map_size*map_size, 250);
% G(obstacle_idx) = 1;
% G(1,1) = 0; G(map_size,map_size) = 0;
% 
% % 防止对角穿越
% for i = 2:map_size
%     for j = 2:map_size
%         % 左上->右下
%         if G(i-1,j-1)==1 && G(i,j)==1 && (G(i-1,j)==0 || G(i,j-1)==0)
%             G(i-1,j) = 1;  % 上
%             G(i,j-1) = 1;  % 左
%         end
%         % 右上->左下
%         if G(i-1,j)==1 && G(i,j-1)==1 && (G(i-1,j-1)==0 || G(i,j)==0)
%             G(i-1,j-1) = 1;  % 左上
%             G(i,j) = 1;      % 右下
%         end
%     end
% end
% G(1,1) = 0; G(map_size,map_size) = 0; % 再次保证起终点畅通
% if     map_size == 20
%        G = zeros(20,20);
%        obstacle_idx = randperm(20*20,62);
%        G(obstacle_idx) = 1;
%        G(1,1) = 0; G(20,20) = 0;
% elseif map_size == 30
%        % 30x30 地图，障碍物随机生成
%         % rng(1); % 固定种子，便于复现
%        G = zeros(30,30);
%        % 随机布置障碍物，障碍密度约0.18,162
%        obstacle_idx = randperm(30*30,135);
%        G(obstacle_idx) = 1;
%        % 确保起点终点可达
%        G(1,1) = 0; G(30,30) = 0;
% elseif map_size == 50
%        % 50x50 地图，障碍物随机生成
%        % rng(2); % 固定种子，便于复现
%        G = zeros(50,50);
%        % 随机布置障碍物，障碍密度约0.18,450
%        obstacle_idx = randperm(50*50, 150);
%        G(obstacle_idx) = 1;
%        G(1,1) = 0; G(50,50) = 0;
% else
%        error('不支持的地图尺寸，请设为20、30或50');
% end
if map_size == 20
    % 20x20 地图
    G = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0;
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0];
elseif map_size == 30
    G = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 1 1 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0;
         0 0 0 0 0 0 1 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 0;
         0 0 0 0 0 0 1 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 0 1 1 1 1 1 1 1 1 1 1 0 0 0;
         0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 1 0 1 1 1 1 1 1 1 1 1 1 0 0 0;
         0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 1 0 1 1 1 1 1 1 1 1 1 1 0 0 0;
         0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
elseif map_size == 50
   G = [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0;
         0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0;
         0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0;
         0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0;
         0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0; 
         0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
else
        error('不支持的地图尺寸，请设为20、30或50');
end

% 地图上下翻转（用于显示符合常规笛卡尔坐标）
num = size(G,1);
for i = 1:num/2  
    for j = 1:num
        m = G(i,j);
        n = G(num+1-i,j);
        G(i,j) = n;
        G(num+1-i,j) = m;
    end
end

%% 起点与终点设置
S = [1 1];             % 起点坐标（行, 列）
E = [num num];         % 终点坐标（行, 列）
G0 = G;                % 保留原始地图
G = G0(S(1):E(1),S(2):E(2)); % 剪裁为有效区域
[Xmax, dimensions] = size(G); 
X_min = 1;         
dimensions = dimensions - 2;   % 路径中间节点数，-2

%% PSO算法参数设置
max_gen = 100;           % 最大迭代次数
num_population = 20;       % 种群数量

fobj = @(x)fitness(x, G); % 适应度函数，需根据路径及地图定义
%%MPSO

tic
[Best_score, Best_pos, MPSO_curve] = MPSO(num_population, max_gen, X_min, Xmax, dimensions, fobj, G);
% [Best_score, Best_pos, PSO_curve] = PSO(num_population, max_gen, X_min, Xmax, dimensions, fobj, G);
%fobj：要最小化或最大化的目标函数句柄。G：一些附加参数，可能是控制参数或约束。

% 结果分析与路径生成
Best_pos = round(Best_pos);      % 将得到的最优节点四舍五入为整数
disp(['MPSO算法寻优得到的最短路径是：', num2str(Best_score)])
route = [S(1) Best_pos E(1)];    % 拼接起点、中间节点、终点
path_MPSO = generateContinuousRoute(route, G);         % 生成连续路径
path_MPSO = GenerateSmoothPath(path_MPSO, G);           % 路径一次平滑
path_MPSO = GenerateSmoothPath(path_MPSO, G);           % 路径二次平滑（可多次）
%%HCLDMS_PSO
[Best_score, Best_pos, HCLDMS_PSO_curve] = HCLDMS_PSO(num_population, max_gen, X_min, Xmax, dimensions, fobj, G);
% 结果分析与路径生成
Best_pos = round(Best_pos);      % 将得到的最优节点四舍五入为整数
disp(['HCLDMS_PSO算法寻优得到的最短路径是：', num2str(Best_score)])
route = [S(1) Best_pos E(1)];    % 拼接起点、中间节点、终点
path_HCLDMS_PSO = generateContinuousRoute(route, G);         % 生成连续路径
path_HCLDMS_PSO = GenerateSmoothPath(path_HCLDMS_PSO, G);           % 路径一次平滑
path_HCLDMS_PSO = GenerateSmoothPath(path_HCLDMS_PSO, G);           % 路径二次平滑（可多次）
%%CPSOS
% [Best_score, Best_pos, CPSOS_curve] = CPSOS(num_population, max_gen, X_min, Xmax, dimensions, fobj, G);
% % 结果分析与路径生成
% Best_pos = round(Best_pos);      % 将得到的最优节点四舍五入为整数
% disp(['CPSOS算法寻优得到的最短路径是：', num2str(Best_score)])
% route = [S(1) Best_pos E(1)];    % 拼接起点、中间节点、终点
% path_CPSOS = generateContinuousRoute(route, G);         % 生成连续路径
% path_CPSOS = GenerateSmoothPath(path_CPSOS, G);           % 路径一次平滑
% path_CPSOS = GenerateSmoothPath(path_CPSOS, G);           % 路径二次平滑（可多次）

%%PSO
toc
tic
[Best_score, Best_pos, PSO_curve] = PSO(num_population, max_gen, X_min, Xmax, dimensions, fobj, G);
Best_pos = round(Best_pos);      % 将得到的最优节点四舍五入为整数
disp(['PSO算法寻优得到的最短路径是：', num2str(Best_score)])
route = [S(1) Best_pos E(1)];    % 拼接起点、中间节点、终点
path_PSO = generateContinuousRoute(route, G);         % 生成连续路径
path_PSO = GenerateSmoothPath(path_PSO, G);           % 路径一次平滑
path_PSO = GenerateSmoothPath(path_PSO, G);           % 路径二次平滑（可多次）
%%DBO
toc
tic
[Best_score, Best_pos, DBO_curve] = DBO(num_population, max_gen, X_min, Xmax, dimensions, fobj,G);
Best_pos = round(Best_pos);      % 将得到的最优节点四舍五入为整数
disp(['DBO算法寻优得到的最短路径是：', num2str(Best_score)])
route = [S(1) Best_pos E(1)];    % 拼接起点、中间节点、终点
path_DBO = generateContinuousRoute(route, G);         % 生成连续路径
path_DBO = GenerateSmoothPath(path_DBO, G);           % 路径一次平滑
path_DBO = GenerateSmoothPath(path_DBO, G);           % 路径二次平滑（可多次）
%%GWO
toc
tic
[Best_score, Best_pos, GWO_curve] = GWO(num_population, max_gen, X_min, Xmax, dimensions, fobj,G);
Best_pos = round(Best_pos);      % 将得到的最优节点四舍五入为整数
disp(['GWO算法寻优得到的最短路径是：', num2str(Best_score)])
route = [S(1) Best_pos E(1)];    % 拼接起点、中间节点、终点
path_GWO = generateContinuousRoute(route, G);         % 生成连续路径
path_GWO = GenerateSmoothPath(path_GWO, G);           % 路径一次平滑
path_GWO = GenerateSmoothPath(path_GWO, G);           % 路径二次平滑（可多次）
toc
% %%SSA
% % [Best_score, Best_pos, SSA_curve] = SSA(num_population, max_gen, X_min, Xmax, dimensions, fobj,G);
% % Best_pos = round(Best_pos);      % 将得到的最优节点四舍五入为整数
% % disp(['SSA算法寻优得到的最短路径是：', num2str(Best_score)])
% % route = [S(1) Best_pos E(1)];    % 拼接起点、中间节点、终点
% % path_SSA = generateContinuousRoute(route, G);         % 生成连续路径
% % path_SSA = GenerateSmoothPath(path_SSA, G);           % 路径一次平滑
% % path_SSA = GenerateSmoothPath(path_SSA, G);           % 路径二次平滑（可多次）
% % 如需对比DE算法，取消注释下行
 % [Best_score, Best_pos, DE_curve] = DE(num_population, max_gen, X_min, Xmax, dimensions, fobj, G);

%% 画收敛曲线
figure(1)
plot(MPSO_curve, 'b-*')
hold on;
plot(PSO_curve, 'y-*')
hold on;
plot(DBO_curve, 'r-*')
hold on;
plot(GWO_curve, 'g-*')
hold on;
plot(HCLDMS_PSO_curve, 'p-*')
hold on;
% plot(CPSOS_curve, 'm-*')
% hold on;
% plot(SSA_curve, 'r-*')
% hold on;
% plot(MPSO_curve, 'b-^')
% hold on;
% plot(PSO_curve, 'y-p')
% hold on;
% plot(DBO_curve, 'm-')
% hold on;
% plot(GWO_curve, 'c-s')
% hold on;
legend('MPSO','HCLDMS_PSO','PSO','DBO','GWO')
% legend('MPSO','HCLDMS_PSO','CPSOS','PSO','DBO','GWO')
% legend('MPSO')
title('复杂路径下算法的收敛曲线')
set(gcf, 'color', 'w')

%% 画路径地图
figure(2)
% 再次翻转地图以正确显示
for i = 1:num/2  
    for j = 1:num
        m = G(i,j);
        n = G(num+1-i,j);
        G(i,j) = n;
        G(num+1-i,j) = m;
    end
end

n = num;
for i = 1:num
    for j = 1:num
        if G(i,j) == 1 
            % 绘制障碍物方格
            x1 = j-1; y1 = n-i; 
            x2 = j;   y2 = n-i; 
            x3 = j;   y3 = n-i+1; 
            x4 = j-1; y4 = n-i+1; 
            p = fill([x1, x2, x3, x4], [y1, y2, y3, y4], [0, 0, 0]); %障碍物颜色
            p.LineStyle = 'none';
            hold on 
        else 
            % 绘制自由格子
            x1 = j-1; y1 = n-i; 
            x2 = j;   y2 = n-i; 
            x3 = j;   y3 = n-i+1; 
            x4 = j-1; y4 = n-i+1; 
            p = fill([x1, x2, x3, x4], [y1, y2, y3, y4], [1, 1, 1]); 
            p.LineStyle = 'none';
            hold on 
        end 
    end 
end 
hold on

hMPSO = drawPath(path_MPSO, 'b');   % 绘制最优路径
hold on
hHCLDMS_PSO = drawPath(path_HCLDMS_PSO, 'p');   % 绘制最优路径
hold on
% hCPSOS = drawPath(path_CPSOS, 'm');
% hold on
hPSO = drawPath(path_PSO, 'y');
hold on
hDBO = drawPath(path_DBO, 'r');
hold on
hGWO = drawPath(path_GWO, 'g');
hold on

legend([hMPSO,hHCLDMS_PSO,hCPSOS,hPSO,hDBO,hGWO], {'MPSO','HCLDMS_PSO','CPSOS','PSO','DBO','GWO'});
legend([hMPSO,hHCLDMS_PSO,hPSO,hDBO,hGWO], {'MPSO','HCLDMS_PSO','PSO','DBO','GWO'});
% legend(hMPSO, 'MPSO');
set(gcf, 'color', 'w')

% toc     SSA麻雀搜索算法，GWO灰狼算法，DBO蜣螂算法
