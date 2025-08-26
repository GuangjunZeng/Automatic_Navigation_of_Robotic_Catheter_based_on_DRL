data1 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\train1.xlsx'); 
indice1 = 22:25:size(data1, 2); 
data1 = data1(: , indice1);

data2 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\train2.xlsx');
indice2 = 22:25:size(data2, 2);  
data2 = data2(: , indice2);

data3 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\train3.xlsx');
indice3 = 22:25:size(data3, 2);  
data3 = data3(: , indice3);

data4 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\train4.xlsx');
indice4 = 22:25:size(data4, 2);  
data4 = data4(: , indice4);

data5 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\train5.xlsx');
indice5 = 22:25:size(data5, 2);  
data5 = data5(: , indice5);

data = [data1, data2, data3, data4, data5];
data(1, :) = []; %将第一行去除
%对所有数据进行归一化: 对每一行的数据进行归一化处理
min_data = min(data, [], 2);
max_data = max(data, [], 2);
normalize_data = (data - min_data) ./ (max_data - min_data);
data = normalize_data;

col_num = size(data, 2);
rand_index = randperm(col_num); %随机排列向量，接下来用来划分训练集和验证集
select_num = floor(col_num*0.8);
idx = rand_index(1:select_num);
train_data = data(:,idx);
idx = rand_index(select_num+1:end);
validation_data = data(:,idx);


%构建模型%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
motion_data = train_data(9:10, : ); %start_point的数据会干扰训练准确
magnet_data = train_data(1:3, : );
inputdata = motion_data;
T = magnet_data(1, : );
dr = magnet_data(2, : );
pr = magnet_data(3, : );
outputdata = magnet_data;
fprintf('T = ');
fprintf('%f ', T);  % 打印每个元素，保留小数点后6位


max_T = max(T);
max_dr = max(dr);
max_pr = max(pr);
disp(['train_data_input_max_T = ' num2str(max_T)]);
disp(['train_data_input_max_dr = ' num2str(max_dr)]);
disp(['train_data_input_max_pr = ' num2str(max_pr)]);


net =  feedforwardnet([20 25 35 20]); 
%net =  feedforwardnet([20 30 37 20]);
%net =  feedforwardnet([23 30 40 25]);
net.layers{1}.transferFcn = 'radbas';
net.layers{2}.transferFcn = 'radbas';
net.layers{3}.transferFcn = 'radbas';
net.layers{4}.transferFcn = 'radbas';
%开始训练
net = train(net, inputdata, outputdata);


%首先计算训练集的误差%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
train_output = net(inputdata);
train_output_T = train_output(1, : );
train_output_dr = train_output(2, : );
train_output_pr = train_output(3, : );

n_train = length(T);
train_mse = sum((T - train_output_T).^2) / n_train;
train_mse2 = sum((dr - train_output_dr).^2) / n_train;
train_mse3 = sum((pr - train_output_pr).^2) / n_train;

% 打印训练集的 MSE
disp(['train_mse_T = ' num2str(train_mse)]);
disp(['train_mse_dr = ' num2str(train_mse2)]);
disp(['train_mse_pr = ' num2str(train_mse3)]);

relative_train_mse_T = train_mse/max_T;
relative_train_mse_dr = train_mse/max_dr;
relative_train_mse_pr = train_mse/max_pr;

disp(['relative_train_mse_T = ' num2str(relative_train_mse_T)]);
disp(['relative_train_mse_dr = ' num2str(relative_train_mse_dr)]);
disp(['relative_train_mse_pr = ' num2str(relative_train_mse_pr)]);



%验证训练模型的准确度，判断有没有过拟合%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
validation_data;
validation_input  = validation_data(9:10, : ); 
validation_input_y = validation_input(2, : );
real_output = validation_data(1:3, : );
real_output_T = validation_data(1, : );
real_output_dr = validation_data(2, : );
real_output_pr = validation_data(3, : );

validation_output = net(validation_input); %net是训练后的model
validation_output_T = validation_output(1, : );
validation_output_dr = validation_output(2, : );
validation_output_pr = validation_output(3, : );


% 假设 real_output_T 和 validation_output_T 已经定义
n = length(real_output_T);
val_mse = sum((real_output_T - validation_output_T).^2) / n;
val_mse2 = sum((real_output_dr - validation_output_dr).^2) / n;
val_mse3 = sum((real_output_pr - validation_output_pr).^2) / n;
disp(['val_mse_T = ' num2str(val_mse)]);
disp(['val_mse_dr = ' num2str(val_mse2)]);
disp(['val_mse_pr = ' num2str(val_mse3)]);

%计算误差的百分比
validation_input_origin = validation_input .* (max_data(9:10, : ) - min_data(9:10, : )) + min_data(9:10, : );
validation_output_origin = validation_output .* (max_data(1:3, : ) - min_data(1:3, : )) + min_data(1:3, : );
real_output_origin = real_output .* (max_data(1:3, : ) - min_data(1:3, : )) + min_data(1:3, : );
validation_input_origin_y = validation_input_origin(2, : );
real_output_origin_T = real_output_origin(1, : );
real_output_origin_dr = real_output_origin(2, : );
real_output_origin_pr = real_output_origin(3, : );
validation_output_origin_T = validation_output_origin(1, : );
validation_output_origin_dr = validation_output_origin(2, : );
validation_output_origin_pr = validation_output_origin(3, : );

figure;
scatter(validation_input_origin_y, real_output_origin_T , 'b', 'filled');
hold on; % 保持绘图状态
scatter(validation_input_origin_y, validation_output_origin_T, 'r', 'filled');
hold off; % 结束绘图状态
title('Scatter Plot of Real ‘T’ vs. Predicted Output ‘T’');

figure;
scatter(validation_input_origin_y, real_output_origin_dr , 'b', 'filled');
hold on; % 保持绘图状态
scatter(validation_input_origin_y, validation_output_origin_dr, 'r', 'filled');
hold off; % 结束绘图状态
title('Scatter Plot of Real ‘dr’ vs. Predicted Output ‘dr’');

figure;
scatter(validation_input_origin_y, real_output_origin_pr , 'b', 'filled');
hold on; % 保持绘图状态
scatter(validation_input_origin_y, validation_output_origin_pr, 'r', 'filled');
hold off; % 结束绘图状态
title('Scatter Plot of Real ‘pr’ vs. Predicted Output ‘pr’');




