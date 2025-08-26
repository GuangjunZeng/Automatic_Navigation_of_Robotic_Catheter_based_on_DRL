%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data1 = readmatrix('C:\Users\admin\Desktop\Labview2018\train1.xlsx'); % replace with your file path
indice1 = 98:100:size(data1, 2);  % 每隔100列选择一列，从第98列开始
data1 = data1(: , indice1);
data2 = readmatrix('C:\Users\admin\Desktop\Labview2018\train2.xlsx');
indice2 = 98:100:size(data2, 2);  % 每隔100列选择一列，从第98列开始
data = [data1, data2];
data(1, :) = [];
data = data'; %注意行列置换

motion_data = data(:, 3:end);
magnet_data = data(:, 1:2);



% XOR data with arbitrary number of input and output parameters
num_input_params = 3;
num_output_params = 2;


P = motion_data'; % input data 注意行列置换
row_std = std(P, 0, 2);
avg_std = mean(row_std);
T = magnet_data'; % output data  注意行列置换




% RBF parameter
goal = 0.1;    %目标性能（误差）
spread = 0.6;  %RBF的扩散参数
hiddenLayerSize = 800; % Number of RBF neurons


% RBF layer
net_rbf = newrb(P, T, 0.1, spread, hiddenLayerSize); %使用RBF

% Get the output of the RBF layer
RBF_output = net_rbf(P);

% BP Neural Network
num_hidden_neurons = 4; 
%需要更精细的网络结构控制时，使用newff函数
net_bp = feedforwardnet(num_hidden_neurons); %creates a fully connected feedforward neural network.
net_bp.trainParam.epochs = 10000; %instructing the BP network to perform 10000 iterations over the training data 
net_bp = train(net_bp, RBF_output, T); %RBF_output is used as the input to the BP network

% Combine the two networks
combined_net = stack(net_rbf, net_bp); % creates a new neural network by stacking one network on top of the other


% Save the trained network to a file
save('RBF_BP_train.mat', 'combined_net');


