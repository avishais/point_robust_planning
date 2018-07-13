clear all
clc
%% Get data

D = dlmread('costBM.txt');

Q = cell(1,4);
s = ones(1,4);
cm = 1;
prev_inx = 1;
for i = 2:size(D,1)
    if all(D(i,2:end)==0)
        Q{s(cm), cm} = D(prev_inx+1:i-1,:); 
        s(cm) = s(cm) + 1;
        
        cm = -D(i,1);
        prev_inx = i;
    end    
end
Q{s(cm), cm} = D(prev_inx+1:end,:); 

%% Proccess
% minimum cost for each method
min_cost = ones(1,4)*1e6;
for i = 1:size(Q,1)
    for j = 1:size(Q,2)
        if ~isempty(Q{i,j}) && Q{i,j}(end,3) < min_cost(j)
            min_cost(j) = Q{i,j}(end,3);
        end
    end
end
disp('Min. cost reached for each methods: ');
disp(min_cost);

% Avg. minimum cost for each method
avg_min_cost = zeros(1,4);
for j = 1:size(Q,2)
    c = 0;
    for i = 1:size(Q,1)
        if ~isempty(Q{i,j})
            avg_min_cost(j) = avg_min_cost(j) + Q{i,j}(end,3);
            c = c + 1;
        end
    end
    avg_min_cost(j) = avg_min_cost(j) / c;
end
disp('Avg. min. cost for each methods: ');
disp(avg_min_cost);

% Average Number of iterations to go below B
B = 500;
num_iter = zeros(1,4);
suc = zeros(1,4);
for j = 1:size(Q,2)
    sum = 0;
    c = 0;
    for i = 1:size(Q,1)
        if ~isempty(Q{i,j})
            M = Q{i,j};
            k = min(M(M(:,3)<B,1));
            if ~isempty(k)
                sum = sum + k;
                c = c + 1;
                suc(j) = suc(j) + 1;
            end
        end
    end
    suc(j) = suc(j) / size(Q,1);
    num_iter(j) = sum / c;
end
disp(['Percent success reaching below cost ' num2str(B) ':']);
disp(suc*100);
disp(['Average Number of iterations to reach below cost ' num2str(B) ':']);
disp(num_iter);

% Avg. minimum cost for each method
avg_prob = zeros(1,4);
for j = 1:size(Q,2)
    c = 0;
    for i = 1:size(Q,1)
        if ~isempty(Q{i,j})
            avg_prob(j) = avg_prob(j) + Q{i,j}(end,5);
            c = c + 1;
        end
    end
    avg_prob(j) = avg_prob(j) / c;
end
disp('Avg. prob. for each methods: ');
disp(avg_prob);
