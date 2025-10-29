function [gBestScore , gBest, cg_curve ] = DBO(N, iter,lb,ub,dim,y,G  ) % 定义DBO算法主函数
   P_percent = 0.7;    % 生产者个体占总数量的百分比

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pNum = round( N *  P_percent );    % 计算生产者个体数量

lb= lb.*ones( 1,dim );    % 下界扩展为维度相同的向量
ub= ub.*ones( 1,dim );    % 上界扩展为维度相同的向量
%Initialization
for i = 1 : N
    x( i, : ) = lb + (ub - lb) .* rand( 1, dim );  % 初始化每个个体随机位置
    fit( i ) = y( x( i, : ) ) ;                    % 计算每个个体的适应度
end

pFit = fit;                       % 记录个体历史最优适应度
pX = x;                           % 记录个体历史最优位置
XX=pX;                            % 记录上一代个体位置
[ gBestScore, bestI ] = min( fit );      % 得到初始全局最优适应度
gBest = x( bestI, : );             % 得到初始全局最优位置

 % Start updating the solutions.
for t = 1 : iter    
        [fmax,B]=max(fit);          % 找到当前种群最差适应度
        worse= x(B,:);              % 最差个体的位置
        r2=rand(1);                 % 产生一个随机数用于分支

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 1 : pNum    
        if(r2<0.8)                              % 大概率选择此分支
            r1=rand(1);                         % 随机数（未用）
          a=rand(1,1);                          % 随机方向
          if (a>0.1)
           a=1;                                 % 绝大多数设置为1
          else
           a=-1;                                % 小概率为-1
          end
    x( i , : ) =  pX(  i , :)+0.3*abs(pX(i , : )-worse)+a*0.1*(XX( i , :)); % 公式(1)：生产者更新
       else
           aaa= randperm(180,1);                % 随机角度
           if ( aaa==0 ||aaa==90 ||aaa==180 )
            x(  i , : ) = pX(  i , :);          % 特殊角度不更新
           end
         theta= aaa*pi/180;                     % 角度转弧度
       x(  i , : ) = pX(  i , :)+tan(theta).*abs(pX(i , : )-XX( i , :));    % 公式(2)：生产者特殊角度更新
        end
      
        x(  i , : ) = Bounds( x(i , : ), lb, ub );    % 边界处理
        fit(  i  ) = y( x(i , : ) );                  % 重新计算适应度
    end 
 [ fMMin, bestII ] = min( fit );      % 当前最优适应度
  bestXX = x( bestII, : );             % 当前最优位置

 R=1-t/iter;                           % 递减因子
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 Xnew1 = bestXX.*(1-R);                % 公式(3)：最优收缩
     Xnew2 =bestXX.*(1+R);             % 公式(3)：最优扩张
   Xnew1= Bounds( Xnew1, lb, ub );     % 边界处理
   Xnew2 = Bounds( Xnew2, lb, ub );    % 边界处理
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     Xnew11 = gBest.*(1-R);            % 公式(5)：全局最优收缩
     Xnew22 =gBest.*(1+R);             % 公式(5)：全局最优扩张
   Xnew11= Bounds( Xnew11, lb, ub );   % 边界处理
    Xnew22 = Bounds( Xnew22, lb, ub ); % 边界处理
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    for i = ( pNum + 1 ) :12                  % 公式(4)：部分种群更新
     x( i, : )=bestXX+((rand(1,dim)).*(pX( i , : )-Xnew1)+(rand(1,dim)).*(pX( i , : )-Xnew2));
   x(i, : ) = Bounds( x(i, : ), Xnew1, Xnew2 );     % 在新边界内裁剪
  fit(i ) = y(  x(i,:) ) ;                         % 计算适应度
   end
  for i = 13: 19                  % 公式(6)：另外一部分种群更新
        x( i, : )=pX( i , : )+((randn(1)).*(pX( i , : )-Xnew11)+((rand(1,dim)).*(pX( i , : )-Xnew22)));
       x(i, : ) = Bounds( x(i, : ),lb, ub);          % 边界处理
       fit(i ) = y(  x(i,:) ) ;                     % 计算适应度
  end
  for j = 20 : N                 % 公式(7)：剩余种群更新
       x( j,: )=gBest+randn(1,dim).*((abs(( pX(j,:  )-bestXX)))+(abs(( pX(j,:  )-gBest))))./2;
      x(j, : ) = Bounds( x(j, : ), lb, ub );        % 边界处理
      fit(j ) = y(  x(j,:) ) ;                      % 计算适应度
  end
   % Update the individual's best fitness vlaue and the global best fitness value
     XX=pX;                                         % 记录上一代个体
    for i = 1 : N 
        if ( fit( i ) < pFit( i ) )                 % 个体适应度更优则更新个体最优
            pFit( i ) = fit( i );
            pX( i, : ) = x( i, : );
        end
        if( pFit( i ) < gBestScore )                % 个体最优优于全局最优则更新全局最优
           gBestScore= pFit( i );
            gBest = pX( i, : );
        end
    end
     cg_curve(t)=gBestScore;                        % 记录每代全局最优
end

% Application of simple limits/bounds
function s = Bounds( s, Lb, Ub)                 % 边界处理函数
  % Apply the lower bound vector
  temp = s;                                     % 临时变量
  I = temp < Lb;                                % 找到小于下界的元素
  temp(I) = Lb(I);                              % 小于下界的赋值为下界
  % Apply the upper bound vector 
  J = temp > Ub;                                % 找到大于上界的元素
  temp(J) = Ub(J);                              % 大于上界的赋值为上界
  % Update this new move 
  s = temp;                                     % 返回裁剪后向量
function S = Boundss( SS, LLb, UUb)             % 备用边界处理函数
  % Apply the lower bound vector
  temp = SS;                                    % 临时变量
  I = temp < LLb;                               % 找到小于下界的元素
  temp(I) = LLb(I);                             % 小于下界的赋值为下界
  % Apply the upper bound vector 
  J = temp > UUb;                               % 找到大于上界的元素
  temp(J) = UUb(J);                             % 大于上界的赋值为上界
  % Update this new move 
  S = temp;                                     % 返回裁剪后向量
%---------------------------------------------------------------------------------------------------------------------------