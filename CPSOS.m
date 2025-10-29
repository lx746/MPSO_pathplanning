function [gBestScore,gBest,cg_curve]=CPSOS(N,iter,lb,ub,dim,y,G)
% PSO参数初始化
Vmax=ones(1,dim).*(ub-lb).*0.15;    %-2.85<V<2.85       % 速度最大值，每一维的最大速度为上下界差的15%
noP=N;                                     % 粒子数量
w=0.8;                                     % 惯性权重
% w_max=0.9;
% w_min=0.4;
% l=0;
c1=1.2;                                    % 个体学习因子
c2=1.2;                                    % 社会学习因子
% c1=1.8;                                    % 个体学习因子
% c2=1.8;                                    % 社会学习因子

% 初始化速度、个体最优、全局最优、收敛曲线
vel=zeros(noP,dim);                        % 所有粒子的速度矩阵，初始化为0
pBestScore=zeros(noP,1);                   % 每个粒子的历史最优适应度值，一维列向量。
pBest=zeros(noP,dim);                      % 每个粒子的历史最优位置，二维矩阵。
gBest=zeros(1,dim);                        % 全局最优粒子的位置，所有粒子历史最优中的最优位置。
cg_curve=zeros(1,iter);                    % 收敛曲线，记录每次迭代的全局最优适应度值，用于观察收敛过程。
% r=zeros(1,iter+1);
% w=zeros(1,iter);
% r(1) = rand();     % 初始值

% 随机初始化粒子位置，确保在允许的自由栅格上
for i = 1:N
    for j = 1:dim
       column = G(:,j+1);                  % 取地图G的第j+1列
       id = find(column == 0);             % 找到该列所有自由栅格的位置（值为0）
       pos(i,j) =  id(randi(length(id)));  % 随机选择一个自由栅格索引作为粒子在该维的起始位置
       % 所有粒子的当前位置矩阵，大小为N × dim。
       id = [];                            % 清空临时索引数组
    end
end

% 个体最优适应度初始化为无穷大（极小化问题）
for i=1:noP
    pBestScore(i,1)=inf;
end

% 全局最优适应度初始化为无穷大
gBestScore=inf;           %全局最优适应度，所有粒子历史最优中的最优值。

% 主循环，进行iter次迭代
for l=1:iter
% while l<iter
    % w(l+1)=w_max-(w_max-w_min)*l/iter;
    
    for i=1:size(pos,1)
        Flag4ub=pos(i,:)>ub;               % 超上界标记
        Flag4lb=pos(i,:)<lb;               % 超下界标记
        pos(i,:)=(pos(i,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;  % 拉回边界
    end
    
    % 计算每个粒子的适应度
    for i=1:size(pos,1)
        fitness= y(pos(i,:));              % 计算第i个粒子的适应度
        if(pBestScore(i)>fitness)          % 如果当前适应度优于历史最优
            pBestScore(i)=fitness;         % 更新个体历史最优适应度
            pBest(i,:)=pos(i,:);           % 更新个体历史最优位置
        end
        if(gBestScore>fitness)             % 如果当前适应度优于全局历史最优
            gBestScore=fitness;            % 更新全局最优适应度
            gBest=pos(i,:);                % 更新全局最优位置
        end
    end
    
    %改进点2：随机学习策略 
    
    % 更新粒子速度和位置  
    % fx = zeros(noP,1);     %新加的
    % for i=1:noP
    %     fx(i) = y(pos(i,:));
    % end

    for i=1:size(pos,1)
        for j=1:size(pos,2)
            % PSO速度更新公式
            % vel(i,j)=w(l)*vel(i,j) + ...      %
            %          c1*rand()*(pBest(i,j)-pos(i,j)) + ... % 个体学习项
            %          c2*rand()*(gBest(j)-pos(i,j));        % 社会学习项
            vel(i,j)=w*vel(i,j) + ...      %
                     c1*rand()*(pBest(i,j)-pos(i,j)) + ... % 个体学习项
                     c2*rand()*(gBest(j)-pos(i,j));        % 社会学习项
            %改进点2：速度更新和随机主流学习
            % 限制速度在[-Vmax, Vmax]范围内
            if(vel(i,j)>Vmax(j))
                vel(i,j)=Vmax(j);
            end
            if(vel(i,j)<-Vmax(j))
                vel(i,j)=-Vmax(j);
            end
            % 更新位置
            pos(i,j)=pos(i,j)+vel(i,j);       
        end
    end
   

    % 对当前全局最优解gBest进行局部搜索，提高适应度
    gBest = LocalSearch(gBest,ub,G);       % 局部搜索提升全局最优解
    gBestScore = y(gBest);                 % 更新全局最优适应度

    cg_curve(l)=gBestScore;                % 记录当前迭代的全局最优适应度
end

end



