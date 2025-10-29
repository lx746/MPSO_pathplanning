function fx = fitness(x,G)
    S = [1 1];       % 新地图的起点
    E = size(G);     % 新地图的终点
    route = [S(1) x E(1)];
    dim = length(route); %
    nB = 0;        % 粒子路径是否经过障碍的数目
    route=round(route);
    for j = 2 : dim-1
        if route(j)<1 || route(j)>size(G,1) || j<1 || j>size(G,2) %新增2，4行
        fx = 1e6;
        return;
    end
       if G(route(j),j) == 1
           nB = nB + 1;
       end
    end
    if nB == 0     % 
        path=generateContinuousRoute(route,G); % 中点邻域搜索
%         path=shortenRoute(path);
        path=GenerateSmoothPath(path,G);
        path=GenerateSmoothPath(path,G);
        fx = 0;
        for i = 1:size(path,1)-1
            fx = fx + sqrt((path(i+1,1)-path(i,1))^2 + (path(i+1,2)-path(i,2))^2);
        end
    else
    fx = E(1)*E(2) * nB;
    end
