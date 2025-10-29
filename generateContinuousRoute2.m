%%
% generateContinuousRoute 生成连续路径
% 根据优化得到的route节点序列，在地图G上生成连续且可行的详细路径点坐标
%
% 输入参数：
%   route ：优化算法输出的路径节点索引序列
%   G     ：环境地图矩阵（0为自由栅格，1为障碍）
%
% 输出参数：
%   path  ：连续可行路径（每行为坐标点 [行 列]）

% function path = generateContinuousRoute(route, G)
% 
% Xmax = size(G,1);              % 地图行数
% dim = length(route);           % 路径节点数
% 
% path = [route(1) 1];           % 起点坐标，列为1
% j = 1;                         % 当前route节点索引
% f = 1;                         % 迭代计数，防止死循环
% 
% while j ~= dim && f < 2000     % 遍历所有路径节点，防止死循环
%     xa = route(j);             % 当前节点行索引
%     xb = route(j+1);           % 下一个节点行索引
%     h = abs(xa - xb);          % 两相邻节点的行距离（高度差）
% 
%     if h == 0                  % 两节点在同一行
%         path = [path; route(j+1) j+1]; % 直接添加下一个节点
%     elseif h == 1              % 两节点在相邻行
%         path = [path; route(j+1) j+1]; % 直接添加下一个节点
%     else
%         k = 0;                 % 标记本段是否已找到路径
%         while k == 0
%             if xa <= xb        % 起点在终点下方（向上走）
%                 if all(G(xa:xb, j) == 0) % j列xa到xb无障碍
%                     % 直接连线，补全中间点
%                     path = [path; (xa+1:xb-1)' j*ones(h-1,1); route(j+1) j+1];
%                 else
%                     % j列xa到xb间有障碍，寻找第一个障碍点
%                     w = xa;
%                     for k2 = xa:xb
%                         if G(k2, j) == 1
%                             w = k2;
%                             break
%                         end
%                     end
%                     % 检查障碍后，j+1列w到xb是否全通
%                     if all(G(w:xb, j+1) == 0)
%                         % 绕开障碍，先到障碍点，再转向j+1列
%                         path = [path; (xa+1:w-1)' j*ones(w-xa-1,1); (w:xb)' (j+1)*ones(xb-w+1,1)];
%                     else
%                         % 进一步调整route(j+1)位置，尝试可行点
%                         mm = h;
%                         while 1
%                             xb = xa + mm - 1;
%                             if G(xb, j+1) == 0
%                                 route(j+1) = xb;
%                                 break;
%                             end
%                             mm = mm - 1;
%                         end
%                         j = j - 1;
%                         continue;
%                     end
%                 end
%             else                % 起点在终点上方（向下走）
%                 if all(G(xa:xb, j) == 0) % j列xa到xb无障碍
%                     % 直接连线，补全中间点
%                     path = [path; (xa-1:-1:xb+1)' j*ones(h-1,1); route(j+1) j+1];
%                 else
%                     % j列xa到xb间有障碍，寻找第一个障碍点
%                     w = xa;
%                     for k2 = xa:-1:xb
%                         if G(k2, j) == 1
%                             w = k2;
%                             break
%                         end
%                     end
%                     % 检查障碍后，j+1列w到xb是否全通
%                     if all(G(w:-1:xb, j+1) == 0)
%                         % 绕开障碍，先到障碍点，再转向j+1列
%                         path = [path; (xa-1:-1:w+1)' j*ones(xa-w-1,1); (w:-1:xb)' (j+1)*ones(w-xb+1,1)];
%                     else
%                         % 进一步调整route(j+1)位置，尝试可行点
%                         mm = h;
%                         while 1
%                             xb = xa - mm + 1;
%                             if G(xb, j+1) == 0
%                                 route(j+1) = xb;
%                                 break;
%                             end
%                             mm = mm - 1;
%                         end
%                         j = j - 1;
%                         continue;
%                     end
%                 end
%             end
%             k = 1;  % 本段已找到路径，退出while
%         end
%     end
%     j = j + 1;      % 跳到下一个节点
%     f = f + 1;      % 步数加一
% end
% 
% if f >= 2000        % 若超出步数限制，视为失败
%     path = [];
% end
% 
% end
%%
function path_PSO=generateContinuousRoute2(route,G)

Xmax = size(G,1);
dim = length(route);

path_PSO = [route(1) 1];
j=1;
f=1;
while j ~=dim&&f<2000
    xa = route(j);
    xb = route(j+1);
    h = abs(xa - xb); % 两相邻栅格高度差
    if h == 0         % 两栅格位于同一行
        path_PSO = [path_PSO ; route(j+1) j+1];
    elseif h == 1     % 两栅格位于相邻行
        path_PSO = [path_PSO ; route(j+1) j+1];
    else
        k=0;
        while k==0
            if xa <= xb              % xa栅格位于xb栅格的下方
                if all(G(xa:xb,j) == 0)    % 第j行xa到xb全为自由栅格
                    path_PSO = [path_PSO; (xa+1:xb-1)' j*ones(h-1,1);route(j+1) j+1];

                else     % 第j行xa到xb不全为自由栅格
                    w = xa;    % 第w个为xa到xb之间的第一个障碍栅格
                    for k = xa:xb
                        if G(k,j) == 1
                            w = k;
                            break
                        end
                    end
                    if  all(G(w:xb,j+1)==0)    % 第j+1列w到wb全为自由栅格
                         path_PSO = [path_PSO; (xa+1:w-1)' j*ones(w-xa-1,1);(w:xb)' (j+1)*ones(xb-w+1,1)];
                    else
                        mm=h;
                        while 1

                            xb=xa+mm-1;

                            if G(xb,j+1)==0
                                route(j+1)=xb;
                                break;
                            end
                            mm=mm-1;
                        end 
                        j=j-1;
                        continue;
                    end
                end

            else
                if all(G(xa:-1:xb,j) == 0)    % 第j行xa到xb全为自由栅格

                        path_PSO = [path_PSO; (xa-1:-1:xb+1)' j*ones(h-1,1);route(j+1) j+1];

                else     % 第j行xa到xb不全为自由栅格
                    w = xa;    % 第w个为xa到xb之间的第一个障碍栅格
                    for k = xa:-1:xb
                        if G(k,j) == 1
                            w = k;
                            break
                        end
                    end
                    if  all(G(w:-1:xb,j+1)==0)    % 第j+1列w到wb全为自由栅格
                         path_PSO = [path_PSO; (xa-1:-1:w+1)' j*ones(xa-w-1,1);(w:-1:xb)' (j+1)*ones(w-xb+1,1)];
                    else
                        mm=h;
                       while 1

                            xb=xa-mm+1;
                            if G(xb,j+1)==0
                                route(j+1)=xb;
                                break;
                            end
                            mm=mm-1;
                       end 
                       j=j-1;
                       continue;
                    end
                end
            end
            k=1;
        end
    end
    j=j+1;
    f=f+1;
end
if f>=2000
    path_PSO=[];
end