
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
position_data = readmatrix('C:\Users\admin\Desktop\Labview2018\测试视频和图片\position_data.xls'); % replace with your file path
% Compute differences in x, y, and time
delta_x = diff(position_data(:, 2));
delta_y = diff(position_data(:, 3));
delta_t = diff(position_data(:, 1));

linear_velocity = sqrt(delta_x.^2 + delta_y.^2) ./ delta_t;
disp(length(linear_velocity));
direction_linear_velocity = atan2(delta_y, delta_x);
disp(length(direction_linear_velocity));
angular_velocity = diff(direction_linear_velocity) ./ delta_t(1:end-1);
disp(length(angular_velocity));
acceleration = diff(linear_velocity) ./ delta_t(1:end-1);
disp(length(acceleration));

%direction_angular_velocity
delta_vx = diff(delta_x ./ delta_t);
delta_vy = diff(delta_y ./ delta_t);
direction_acceleration = atan2(delta_vy, delta_vx);
disp(length(direction_acceleration));

%motion_data = horzcat(acceleration, linear_velocity, direction_linear_velocity, angular_velocity, direction_acceleration);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
magnet_data = readmatrix('C:\Users\admin\Desktop\Labview2018\测试视频和图片\magnet_data.xls');




% XOR data with arbitrary number of input and output parameters
num_input_params = 3;
num_output_params = 2;

P = rand(num_input_params, 4); % Random input data
T = rand(num_output_params, 4); % Random output data

%disp(P);
%disp(T);
%P = [0 0 0; 1 0 1; 0 1 2; 1 1 3];
%T = [0 1; 1 0; 1 2; 0 3];

% RBF parameters
spread = 0.5;
hiddenLayerSize = 2; % Number of RBF neurons

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


