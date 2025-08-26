% Sample data
% Assuming 3 input parameters and 2 output parameters
P = [0 0 0; 1 0 1; 0 1 2; 1 1 3];
T = [0 1; 1 0; 1 2; 0 3];

% Build RBF network
goal = 0.01; %训练目标的误差
spread = 1;  %RBF神经元的拓展值
net = newrb(P', T', goal, spread);

% Test data
% Assuming 3 input parameters for testing
X_test = [0.5 0.5 1; 0.2 0.8 2];

% Use the trained network for prediction
Y_test = net(X_test');

disp(Y_test);
