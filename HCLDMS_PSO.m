function [position,value,iteration,y,Error,gbestfit]= HCLDMS_PSO(N,iter,lb,ub,dim,y,G) 
% function [gBestScore,gBest,cg_curve]=MPSO(N,iter,lb,ub,dim,y,G)
% 输入参数：
% num_particle：粒子数
% range：变量范围（下界、上界）
% dimension：问题维度
% max_iteration：最大迭代次数
% max_FES：最大适应度评估次数
% func_num：优化函数编号
% 输出参数：
% position：最优解位置
% value：最优适应度值
% iteration：实际迭代次数
% max_FES：实际适应度评估次数
% Error：最优解与真实值误差
% gbestfit：进化过程中群体最优值与真实值的差异

rand('state',sum(100*clock));  % 初始化随机种子
% load fbias_data;               % 加载基准函数偏置数据

% 速度监控变量初始化
check_vel = [];
check_vel1 = [];
check_vel2 = [];

% ================= 参数设置 =================
num_g = N;  % 总粒子数
if dim == 10
    num_g1 = 8;        % CL子群粒子数
elseif dim == 30
    num_g1 = 16;
end
num_g2 = num_g-num_g1;           % DMS子群粒子数
group_ps = 3;                    % 每个子群粒子数
group_num = num_g2/group_ps;     % 子群总数

% CL学习概率曲线
j = 0:(1/(num_g-1)):1;           
j = j*10;
Pc = ones(dim,1)*(0.05+((0.45).*(exp(j)-exp(j(1)))./(exp(j(num_g))-exp(j(1)))));      

% 惯性权重（线性和非线性递减）
Weight = 0.99-(1:iter)*0.79/iter;                                     
Weight1 = 0.99 + (0.2-0.99)*(1./(1 + exp(-5*(2*(1:iter)/iter - 1)))); 
C = 0.15;   % DMS子群非线性递减惯性权重修正常数

% 加速系数c1、c2
c1 = 2.5-(1:iter)*2/iter;                   
c2 = 0.5+(1:iter)*2/iter; 

pm = 0.1;      % DMS子群变异概率
flag = 0;      
gap1 = 5;      
sigmax = 1;    % 高斯变异参数最大值
sigmin = 0.1;  % 高斯变异参数最小值
sig = 1;       % 当前高斯变异参数

% ================= 粒子初始化 =================
VRmin = lb*ones(num_g,dim);   % 每个粒子的下界
VRmax = ub*ones(num_g,dim);   % 每个粒子的上界
interval = VRmax-VRmin;                   % 搜索空间区间
v_max = 0.5 * interval;                   % 速度最大值
v_min = -v_max;                           % 速度最小值
pos = VRmin+ interval.*rand(num_g,dim);     % 粒子初始位置
vel = v_min+(v_max-v_min).*rand(num_g,dim); % 粒子初始速度

k=0;      % 迭代计数器
fitcount=0; % 适应度评估计数器

% 计算初始适应度
result = zeros(num_g,1);
for i = 1:num_g
    result(i) = y(pos(i,:));
end
fitcount = fitcount + num_g;              % 累加评估次数
pbest_pos = pos;                          % 初始化个体最优位置
pbest_val = result';                      % 初始化个体最优值

[gbest_val,g_index] = min(result);        % 初始化群体最优值和位置
gbest_pos = pos(g_index,:); 
g_res(1:fitcount) = gbest_val;      % 记录进化过程群体最优值

obj_func_slope=zeros(num_g,1);            % 每个粒子停滞计数
fri_best=(1:num_g1)'*ones(1,dim);   % CL子群学习伙伴索引

% =============== 初始化CL子群学习伙伴 ===============
for i = 1:num_g1
    fri_best(i,:) = i*ones(1,dim);      % 初始设为自身
    friend1 = ceil(num_g*rand(1,dim));  % 随机选择第一个朋友
    friend2 = ceil(num_g*rand(1,dim));  % 随机选择第二个朋友
    % 选择适应度更优的朋友作为学习对象
    friend = (pbest_val(friend1)<pbest_val(friend2)).*friend1+(pbest_val(friend1)>=pbest_val(friend2)).*friend2;
    toss = ceil(rand(1,dim)-Pc(:,i)');  % 按概率决定是否保留原学习对象
    if toss == ones(1,dim)
       temp_index = randperm(dim);      % 保证至少有一个维度是新学习对象
       toss(1,temp_index(1)) = 0;
       clear temp_index;
    end
    fri_best(i,:) = (1-toss).*friend+toss.*fri_best(i,:); % 更新学习对象索引
    for d = 1:dim
        fri_best_pos(i,d) = pbest_pos(fri_best(i,d),d);    % 获取学习对象的位置
    end
end

% =============== 初始化DMS子群及局部最优 ===============
for i = 1:group_num
    group_id(i,:) = [((i-1)*group_ps+num_g1+1):i*group_ps+num_g1];   % 每组的粒子编号
    pos_group(group_id(i,:)) = i;                                    % 粒子所属分组编号
    [gbestval(i),gbestid] = min(pbest_val(group_id(i,:)));           % 组内局部最优值及索引
    gbest(i,:) = pbest_pos(group_id(i,gbestid),:);                   % 组内局部最优位置
end

count = 0;   index = []; m = 1; 

% =============== 进化主循环 ===============
while k <= iter && fitcount <= y
    % -------- 多样性指标记录 --------
    if m <= iter                                            
        ava_pos = mean(pos);                                          
        Div_whole(m) = PSO_Diversity(pos,ava_pos);                    % 整体多样性
        Div_CL(m) = PSO_Diversity(pos(1:num_g1,:),ava_pos);           % CL子群多样性
        Div_DMS(m) = PSO_Diversity(pos(num_g1+1:num_g,:),ava_pos);    % DMS子群多样性
        m = m + 1;    
    end

    k=k+1;  % 迭代计数+1

    % -------- 计算群体均值和标准差 --------
    Average_g = mean(result);    
    Fit_std_g = std(result);     

    for i = 1:group_num
        Average_n(group_id(i,:)) = mean(result(group_id(i,:)));       
        Fit_std_n(group_id(i,:)) = std(result(group_id(i,:)));        
    end

    % -------- CL子群速度和位置更新 --------
    gbest_pos_temp = repmat(gbest_pos,num_g1,1);  
    delta_g1 = (c1(k).*rand(num_g1,dim).*(fri_best_pos(1:num_g1,:)-pos(1:num_g1,:))) + ...
               (c2(k).*rand(num_g1,dim).*(gbest_pos_temp-pos(1:num_g1,:)));
    vel_g1 = Weight(k)*vel(1:num_g1,:)+delta_g1;
    % 边界处理
    vel_g1 = ((vel_g1<v_min(1:num_g1,:)).*v_min(1:num_g1,:)) + ((vel_g1>v_max(1:num_g1,:)).*v_max(1:num_g1,:)) + ...
             (((vel_g1<v_max(1:num_g1,:))&(vel_g1>v_min(1:num_g1,:))).*vel_g1);
    pos_g1 = pos(1:num_g1,:)+vel_g1;

    % -------- DMS子群速度、位置、变异更新 --------
    for i = num_g1 + 1: num_g
        % 非线性自适应惯性权重
        if Average_n(i) >= Average_g
            wx(i) = Weight1(k) + C;   if wx(i)>0.99,  wx(i) = 0.99;end   
        else     
            wx(i) = Weight1(k) - C;   if wx(i)<0.20,  wx(i) = 0.20;end   
        end
        % 速度更新
        delta_g2(i,:) = (c1(k).*rand(1,dim).*(pbest_pos(i,:)-pos(i,:))) + ...
                        (c2(k).*rand(1,dim).*(gbest(pos_group(i),:)-pos(i,:)));
        vel_g2(i,:) = wx(i)*vel(i,:)+ delta_g2(i,:);    
        % 边界处理
        vel_g2(i,:) = ((vel_g2(i,:)<v_min(i,:)).*v_min(i,:)) + ((vel_g2(i,:)>v_max(i,:)).*v_max(i,:)) + ...
                      (((vel_g2(i,:)<v_max(i,:))&(vel_g2(i,:)>v_min(i,:))).*vel_g2(i,:));
        pos_g2(i,:) = pos(i,:) + vel_g2(i,:);
        % 非均匀变异
        pos_g2(i,:) = Non_uniform_mutation(pos_g2(i,:),pm,k,iter,[lb,ub]);
    end

    % -------- 合并所有子群 --------
    pos_g2(1:num_g1,:) = []; 
    vel_g2(1:num_g1,:) = [];   
    pos=[pos_g1;pos_g2];
    vel=[vel_g1;vel_g2];

    % -------- 边界检查，重新评估适应度 --------
    for i=1:num_g   
        if (sum(pos(i,:)>VRmax(i,:))+sum(pos(i,:)<VRmin(i,:))==0)
           index = [index;i];
        end
    end
    if ~isempty(index)
      for ii = 1:length(index)
          result(index(ii)) = y(pos(index(ii),:));
      end
      index = [];
    end

    % -------- 个体最优更新 --------
    for i=1:num_g   
       if (sum(pos(i,:)>VRmax(i,:))+sum(pos(i,:)<VRmin(i,:))==0)
          fitcount=fitcount+1;
          if fitcount>=y,  break;   end
          if  result(i) < pbest_val(i)
            pbest_pos(i,:) = pos(i,:);   
            pbest_val(i) = result(i);
            obj_func_slope(i) = 0;
          else
            obj_func_slope(i)=obj_func_slope(i)+1;
          end          
          g_res(fitcount) = gbest_val;
       end 
    end

    % -------- 群体最优高斯变异 --------
    [gbestvaltmp,ind2] = min(pbest_val);        
    gbest_postmp = pbest_pos(ind2,:);           
    if gbestvaltmp < gbest_val
        gbest_pos = gbest_postmp;
        gbest_val = gbestvaltmp;
        flag = 0;
    else
        flag = flag+1;
    end
    % 更新DMS组局部最优
    for i = 1:num_g
        if i > num_g1  &&  pbest_val(i) < gbestval(pos_group(i))  
            gbest(pos_group(i),:) = pbest_pos(i,:);
            gbestval(pos_group(i)) = pbest_val(i);
        end
    end
    % 高斯变异操作
    if flag >= gap1
        pt = gbest_pos;
        d1 = unidrnd(dim);    randdata = 2 * rand(1,1)-1;
        pt(d1) = pt(d1)+sign(randdata)*(ub-lb)*normrnd(0,sig^2);  
        pt(find(pt(:)>ub)) = ub * rand;                           
        pt(find(pt(:)<lb)) = lb * rand;
        cv = y(pt);                                     
        fitcount = fitcount+1;
        g_res(fitcount) = cv;
        if cv < gbest_val
            gbest_pos = pt;
            gbest_val = cv;
            flag=0;       
        end           
    end
    sig = sigmax - (sigmax-sigmin)*(fitcount/y);      

    % -------- CL子群学习对象动态更新 --------
    for i=1:num_g1             
        if obj_func_slope(i)>5
            fri_best(i,:)=i*ones(1,dim);          
            friend1=ceil(num_g1*rand(1,dim));     
            friend2=ceil(num_g1*rand(1,dim));
            friend=(pbest_val(friend1)<pbest_val(friend2)).*friend1+(pbest_val(friend1)>=pbest_val(friend2)).*friend2;
            toss=ceil(rand(1,dim)-Pc(:,i)');
            if toss==ones(1,dim)
                temp_index=randperm(dim);
                toss(1,temp_index(1))=0;
                clear temp_index;
            end
            fri_best(i,:)=(1-toss).*friend+toss.*fri_best(i,:);
            for d=1:dim
                fri_best_pos(i,d)=pbest_pos(fri_best(i,d),d);
            end
            obj_func_slope(i)=0;
        end
    end

    % -------- DMS子群动态分组 --------
    change_flag = 0;
    for i = 1:group_num
        flag = 1;
        for kk = 1:group_ps
            flag=flag*(sum(abs(pbest_pos(group_id(i,kk),:)-gbest(i,:))<1e-3)==dim);
        end
        if flag == 1
            change_flag = 1;  break;
        end
    end
    if change_flag == 1 || mod(k,5) == 0    
        rc = randperm(num_g2) + num_g1;           
        group_id=[]; gbest=[]; gbestval=[];
        for i = 1:group_num
            group_id(i,:) = rc(((i-1)*group_ps + 1):i*group_ps);
            pos_group(group_id(i,:)) = i;
            [gbestval(i),gbestid] = min(pbest_val(group_id(i,:)));
            gbest(i,:) = pbest_pos(group_id(i,gbestid),:);
        end
     end   

     % -------- 速度监控 --------
     check_vel=[check_vel (sum(abs(vel'))/dim)'];         
     check_vel1=[check_vel1 sum(check_vel(1:num_g1,end))/num_g1];
     check_vel2=[check_vel2 sum(check_vel(num_g1+1:end,end))/num_g2];

     if fitcount>=y,  break;   end   
     if (k==iter) && (fitcount<y)
        k=k-1;
        count = count + 1;
     end
end

% ================= 输出结果 =================
check_vel1 = check_vel1./(ub-lb);
check_vel2 = check_vel2./(ub-lb);
position = gbest_pos;
value = gbest_val;
iteration = k;
y = fitcount;
Error = value;              % 误差（最优适应度值）
gbestfit = g_res;         % 进化过程中最优值
end

% ================= 非均匀变异操作 =================
function [newpop] = Non_uniform_mutation(pop,pm,t,T,Bound)
b = 2;                
[ps,D]=size(pop);     
VRmin = Bound(1);     
VRmax = Bound(2);     
newpop = pop;
for i = 1:ps
  for j = 1:D
    if rand() < pm
       aa = rand(1,D);
       N_mm = diag(aa); 
       if round(rand()) == 0
           newpop(i,j) = pop(i,j) + N_mm(j,j)*(VRmax - pop(i,j))*(1 - t/T)^b;
       else
           newpop(i,j) = pop(i,j) - N_mm(j,j)*(pop(i,j) - VRmin)*(1 - t/T)^b;
       end
    end
  end
end 
end

% ================= 群体多样性计算 =================
function D = PSO_Diversity(x, ava_Xd)       
[N,D] = size(x);             
ava_Xd = repmat(ava_Xd,N,1);    
Distance = sum([x - ava_Xd].^2,2).^(1/2);
D = mean(Distance);
end