% 处理所有文件并合并数据
data1 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0609 1.xlsx');
data2 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0609 2.xlsx');
data3 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0609 3.xlsx');
data4 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0609 4.xlsx');
data5 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0609 5.xlsx');
data6 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0609 6.xlsx');
data7 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0609 7.xlsx');
data8 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0610 1.xlsx');
data9 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0610 2.xlsx');
data10 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0610 3.xlsx');
data11 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0610 4.xlsx');
data12 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0610 5.xlsx');
data13 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0610 6.xlsx');
data14 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0615 1.xlsx');
data15 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0615 2.xlsx');
data16 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0616 00.xlsx');
data17 = process_data('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\9mT 0616 01.xlsx');

data = [data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11, data12, data13, data14, data15, data16, data17];
data(1, :) = []; 


col_num = size(data, 2);
select_num = floor(col_num * 0.8);
rand_index = randperm(col_num, col_num - select_num); % 随机抽取20%的索引作为验证集
validation_data = data(:, rand_index);
train_data = data(:, setdiff(1:col_num, rand_index)); % 剩余的作为训练集


%构建模型%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
motion_data = train_data(8:9, : ); %start_point的数据会干扰训练准确
magnet_data = train_data([1, 2, 15], : );
start_motion_data = train_data(6:7, : );

inputdata = train_data(20:21, : ); % 去除原第一行后的20,21行

dr = magnet_data(2, : );
pr = magnet_data(3, : );
outputdata = magnet_data(2:3, : );

max_dr = max(dr);
max_pr = max(pr);
disp(['train_data_input_max_dr = ' num2str(max_dr)]);
disp(['train_data_input_max_pr = ' num2str(max_pr)]);


%训练模型%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
net =  feedforwardnet([55 57 55 45]); %似乎三层的训练效果还更好？
%net =  feedforwardnet([20 30 37 20]);
%net =  feedforwardnet([23 30 40 25]);
net.layers{1}.transferFcn = 'poslin';
net.layers{2}.transferFcn = 'radbas';
net.layers{3}.transferFcn = 'radbas';
net.layers{4}.transferFcn = 'radbas';


% 设置学习率
net.trainParam.lr = 0.003;  % learning rate
net.trainParam.goal = 1e-3; % 训练目标误差 
net.trainParam.min_grad = 1e-5; % 最小梯度
net.trainParam.mu_max = 1e14; % 当 mu 较大时，算法更接近梯度下降法，适合远离最优解时稳定更新; 当 mu 较小时，算法更接近高斯-牛顿法，适合接近最优解时快速收敛。
net.trainParam.max_fail = 150;  % 验证失败次数超过 ... 次时停止

%开始训练
net = train(net, inputdata, outputdata);


%首先计算训练集的误差%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
train_output = net(inputdata);
train_output_dr = train_output(1, : );
train_output_pr = train_output(2, : );

n_train = length(dr);

train_mse2 = sum((dr - train_output_dr).^2) / n_train;
train_mse3 = sum((pr - train_output_pr).^2) / n_train;

disp('训练误差:')
disp(['train_mse_dr = ' num2str(train_mse2)]);
disp(['train_mse_pr = ' num2str(train_mse3)]);

train_mean_abs_diff2 = mean(abs(dr - train_output_dr));
train_mean_abs_diff3 = mean(abs(pr - train_output_pr));
disp(['train_mean_abs_diff2_dr = ' num2str(train_mean_abs_diff2)]);
disp(['train_mean_abs_diff3_pr = ' num2str(train_mean_abs_diff3)]);

relative_train_mean_abs_diff2 = mean(abs( (dr - train_output_dr))) / mean(dr);
relative_train_mean_abs_diff3 = mean(abs( (pr - train_output_pr))) / mean(pr);
disp(['relative_train_mean_abs_diff2_dr = ' num2str(relative_train_mean_abs_diff2)]);
disp(['relative_train_mean_abs_diff3_pr = ' num2str(relative_train_mean_abs_diff3)]);

relative_train_smse_dr = sqrt(train_mse2)/max_dr;
relative_train_smse_pr = sqrt(train_mse3)/max_pr;
%disp(['relative_train_smse_dr = ' num2str(relative_train_smse_dr)]);
%disp(['relative_train_smse_pr = ' num2str(relative_train_smse_pr)]);


%验证训练模型的准确度，判断有没有过拟合%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
validation_data;
validation_input  = validation_data(20:21, : ); 
validation_input_relative_y = validation_input(2, : );
real_output =  validation_data([1, 2, 15], : );
real_output_dr = real_output(2, : );
real_output_pr = real_output(3, : );

validation_output = net(validation_input); %net是训练后的model
validation_output_dr = validation_output(1, : );
validation_output_pr = validation_output(2, : );

disp('验证误差:')
% 假设 real_output_T 和 validation_output_T 已经定义
n = length(real_output_dr);
val_mse2 = sum((real_output_dr - validation_output_dr).^2) / n;
val_mse3 = sum((real_output_pr - validation_output_pr).^2) / n;
disp(['val_mse_dr = ' num2str(val_mse2)]);
disp(['val_mse_pr = ' num2str(val_mse3)]);

mean_abs_val_diff2 = mean(abs(real_output_dr - validation_output_dr));
mean_abs_val_diff3 = mean(abs(real_output_pr - validation_output_pr));
disp(['mean_abs_val_diff2 = ' num2str(mean_abs_val_diff2)]);
disp(['mean_abs_val_diff3 = ' num2str(mean_abs_val_diff3)]);


relative_mean_abs_val_diff2 = mean(abs(real_output_dr - validation_output_dr)) / mean(real_output_dr);
relative_mean_abs_val_diff3 = mean(abs(real_output_pr - validation_output_pr)) / mean(real_output_pr);
disp(['relative_mean_abs_val_diff2 = ' num2str(relative_mean_abs_val_diff2)]);
disp(['relative_mean_abs_val_diff3 = ' num2str(relative_mean_abs_val_diff3)]);


figure;
scatter(validation_input_relative_y, real_output_dr , 'b', 'filled');
hold on; % 保持绘图状态
scatter(validation_input_relative_y, validation_output_dr, 'r', 'filled');
hold off; % 结束绘图状态
title('Scatter Plot of Real ‘dr’ vs. Predicted Output ‘dr’');

figure;
scatter(validation_input_relative_y, real_output_pr, 'b', 'filled');
hold on; % 保持绘图状态
scatter(validation_input_relative_y, validation_output_pr, 'r', 'filled');
hold off; % 结束绘图状态
title('Scatter Plot of Real ‘pr’ vs. Predicted Output ‘pr’');



%保存训练的网络到本地一个新的文件中%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save('trained_net_color_2inputs_2outputs.mat', 'net');




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function processed_data = process_data(file_path)
    ori_data = readmatrix(file_path);
    % 计算所有有效块的起始列（从26开始，每隔50列，直到列数-45）
    start_cols = 27:50:(size(ori_data, 2) - 45);
    processed_data = zeros(size(ori_data, 1), length(start_cols));
    for i = 1:length(start_cols)
        cols = start_cols(i) + 41 : start_cols(i) + 45;
        processed_data(:, i) = mean(ori_data(:, cols), 2);
    end
end




