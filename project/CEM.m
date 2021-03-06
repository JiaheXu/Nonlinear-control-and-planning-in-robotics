function [path , crs, Time ] = CEM( map ,path, start, goal,iter,Sigma)
sn = size(path,1) - 2  % total intermediate knots
ng = 1;  % number of Gaussian mixture models

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%iter = 400; % total iterations

S = si_init(map , start , goal , sn ,  ng , path, Sigma);
N = 100;  % total samples
% take top 5%
p =.05;    % rho-quantile
nf = round(p*N);
S.ss = [];
s = feval(S.f_sinit, S);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%crs = inf*ones(1,iter);
crs = inf*ones(1,10000);
Time = zeros(1,10000);

last_cost = 0.0;
uf = 0;   % set to 1 to generate and save separate plots for each iteration
S.uf = uf;
pss = zeros(N, S.n*S.sn);

k = 1;
tmp = s.mu;
for k=1:iter
%while( 1 )
    S.k = k;  
    tic
    j0 = 2;
    if k==1
        j0=1;
    end
    
    for j=j0:N
        % pss :第j轮生成的sn 个随机点
        pss(j,:) = feval(S.f_sample, s, 1, S,k);
        % xs应该是右边示意图的点 是真正路径？
        xs = feval(S.f_traj, pss(j,:), S);
        % xs 存每轮生成的随机点？for what？
      
        size(xs);
        
        S.ss(j).xs = xs;
        %cost 各点之间直线距离之和
        S.ss(j).c = feval(S.f_cost, pss(j,:), S);
        S.ss(j).c;
    end
  
  
  fprintf('finished round %d\n',k);
%  tic
  %把N轮 由Sn个随机点的组成的随机路径排序
  [pss, cs] = ce_sort(pss, S);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  %只按前nf好的sample更新
  
  sn = ce_fit(pss(1:nf,:), s, S);
  %fprintf("mu!!!!\n");
  %norm(tmp - s.mu)
  %s.mu = pss(1,:);
  a = 0.9;
  s.mu = a*sn.mu + (1-a)*s.mu;
  b = 0.9;
  s.Sigma = b*sn.Sigma + (1-b)*s.Sigma;
  xs = feval(S.f_traj, pss(1,:), S);
  crs(k) = cs(1);
  Time(k) = toc;
  cs(1)
       
end
fprintf("orz")
path = xs';
crs = crs(1,1:k);
Time = Time(1,1:k);
collision_check(map, path);
cs(1)


function [pss, cs] = ce_sort(pss, S)
N = size(pss, 1); % number of samples
%cost
cs = zeros(N, 1);
for i=1:N
  cs(i) = feval(S.f_cost, pss(i,:), S);
end
[cs,is] = sort(cs, 1, 'ascend');
pss = pss(is,:);


function s = ce_fit(pss, s, S)

% fit independently each point
if (S.ng == 1) 
    % sn个节点
  for i=1:S.sn
    % extract all samples at i-th point
    ind = (i-1)*S.n + (1:S.n);
    s.mu(ind) = mean(pss(:,ind));
    
%     size(S.rf)
%     size(rand(S.n,1))
%     size(cov(pss(:,ind)))
%     size(s.Sigma)
%     size(diag(S.rf.*rand(S.n,1)))
    
    s.Sigma(ind,ind) = cov(pss(:,ind)) + diag(S.rf.*rand(S.n,1));
  end
else
  if S.k > inf
    s = gmdistribution.fit(pss, S.ng, 'Start', s);
  else
    s = gmdistribution.fit(pss, S.ng, 'Regularize',.01);
  end
end


%%%%%%% problem specific %%%%%%%%%%

function S = si_init(map , start , goal ,sn, ng ,path,Sigma)

%S.sys = 'si';

% problem specific params

% general parameters
S.f_sinit = @si_sinit;
S.f_traj = @si_traj;
S.f_draws = @si_draws;
S.f_sample = @si_sample;
S.f_valid = @si_valid;
S.f_cost = @si_cost;
S.map = map;
S.path = path;
S.Sigma = Sigma;
% dimension
S.n = 3;
% sn knods
S.sn = sn;
% n gaussian model
S.ng = ng;

%initial position and goal position
S.xi = reshape(start , [3,1]);
S.xf = reshape(goal, [3,1]);

%S.xi = [5.5; 1];
S.snf = 6*S.sn;
S.ph = 0;

S.rf = [.1; .1 ; .1];
% n 段路线 画图 起点到终点 sn + 2个点 start line
function xs = stline(xi, xf, sn)
n = length(xi);
xs = zeros(n, sn + 2);
for i=1:n
  xs(i,:) = linspace(xi(i), xf(i), sn+2);
end



function s = si_sinit(S)
% ng个高斯模型
% S.n 维 S.sn个gaussian
s.mu = zeros(S.ng, S.n*S.sn);
s.Sigma = zeros(S.n*S.sn, S.n*S.sn, S.ng);

%xs = stline(S.xi,S.xf, S.sn); 
xs = S.path';
size(xs);
for j=1:S.ng
  s.mu(j,:) = reshape(xs(:,2:end-1), S.n*S.sn, 1);
  %##############################################################################
  s.Sigma(:,:,j) = S.Sigma*eye(S.n*S.sn);
end

s.PComponents = 1/S.sn*ones(S.ng,1);


%每个点用直线连？
% function xs = si_traj(ps, S)
% 
% xs = [S.xi, reshape(ps, S.n, S.sn), S.xf];
% size(xs);
% xs = interp1(linspace(0, 1, size(xs,2)), xs', ...
%              linspace(0, 1, S.snf), 'cubic')';
function path = si_traj(ps, S)
xs = reshape(ps , S.n , length(ps)/S.n  );
xs = [ S.xi , xs , S.xf];
path =xs;

% 按各个高斯分布生成随机点 对应只生成一个点
function ps = si_sample(s, c, S, k)

ps = zeros(1, S.n*S.sn);
while(1)
    last_ind = [];
    for i=1:S.sn
        ind = (i-1)*S.n + (1:S.n);
        while(1)
            ps(ind) = mvnrnd(s.mu(ind), s.Sigma(ind,ind), 1);
            if(k==1)
                ps(ind) = mvnrnd(s.mu(ind), [0.1 0.1 0.1], 1);
            end
            if(collision_check(S.map, ps(ind)) == 0)
                    break
            end
%             if (i==1)
%                 if(collision_check(S.map, ps(ind)) == 0)
%                     break
%                 end
%             else
%                 if( points_path_inside(ps(last_ind),ps(ind) , S) ==0)
%                     break;
%                 end
%             end    
        end
        last_ind = ind;
    end
    flag = feval(S.f_valid, ps, S);
    if(flag == 1)
        fprintf("valid sample\n");
    end
    
    if (c==0 || flag == 1)
        break
    end
end


% ps 是随机生成的sn个点
% sum( , 1)是对每一列进行求和
%cost是n维空间上 各个点之间直线距离之和
function f = si_cost(ps, S)
ps = reshape(ps, S.n, S.sn);
xsa = [S.xi, ps];
xsb = [ps, S.xf];
dxs = xsb - xsa;

f = sum(sqrt(sum(dxs.*dxs, 1)));

function f = si_valid(ps, S)
% f = 1 then valid
% f = 0 then invalid
f = 1;
% xs = ps;
xs = reshape(ps , S.n , length(ps)/S.n  );
ps = [ S.xi , xs , S.xf];

%##################################################
ps';
si_inside(ps, S);
if (si_inside(ps, S) == 1)
    f = 0;
    return
end

function f = si_inside(xs, S)
% f is 0, then collision free
% f is 1, then have collision
xs = xs';
f = 0;
for i = 2:size(xs,1)
    if( subpath_check(S.map , xs(i-1,:) , xs(i,:) ) == 1)
        f = 1;
        break;
    end
end
