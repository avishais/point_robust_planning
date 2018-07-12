Q = dlmread('cost.txt');

T = Q(2:end,1); % Time
C = Q(2:end,2); % Cost
P = Q(2:end,3); % Probability

figure(1)
subplot(121)
plot(T, C ,'o-k', 'linewidth',2);
xlabel('Time (sec)');
ylabel('Cost');


subplot(122)
plot(T,P, 'o-k', 'linewidth',2);
xlabel('Time (sec)');
ylabel('Probability');

