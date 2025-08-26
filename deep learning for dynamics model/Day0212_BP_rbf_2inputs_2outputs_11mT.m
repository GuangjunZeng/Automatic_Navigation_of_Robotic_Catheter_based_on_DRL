ori_data1 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new1.xlsx'); 
indice1 = (25 + 44):50:size(ori_data1, 2); %尽量往后时间段的tip肯定就基本稳定位置
%indice1 = (6275 + 37):50:size(ori_data1, 2);
data1 = ori_data1( : , indice1);
%indice11 = (6275 + 39):50:size(ori_data1, 2); %尽量往后时间段的tip肯定就基本稳定位置
%data11 = ori_data1( : , indice11);

ori_data2 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new2.xlsx'); 
indice2 = (25 + 44):50:size(ori_data2, 2); %尽量往后时间段的tip肯定就基本稳定位置
data2 = ori_data2(: , indice2);
%indice22 = (6275 + 39):50:size(ori_data2, 2); %尽量往后时间段的tip肯定就基本稳定位置
%data22 = ori_data2(: , indice2);

ori_data3 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new3.xlsx'); 
indice3 = (25 + 44):50:size(ori_data3, 2); %尽量往后时间段的tip肯定就基本稳定位置
data3 = ori_data3(: , indice3);

ori_data4 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new4.xlsx'); 
indice4 = (26 + 44):50:size(ori_data4, 2); %尽量往后时间段的tip肯定就基本稳定位置
data4 = ori_data4(: , indice4);

ori_data5 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new5.xlsx'); 
indice5 = (31 + 55):60:size(ori_data5, 2); %尽量往后时间段的tip肯定就基本稳定位置
data5 = ori_data5(: , indice5);

ori_data6 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new6.xlsx'); 
indice6 = (31 + 55):60:size(ori_data6, 2); %尽量往后时间段的tip肯定就基本稳定位置
data6 = ori_data6(: , indice6);

ori_data7 = readmatrix('C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new7.xlsx'); 
indice7 = (31 + 55):60:size(ori_data7, 2); %尽量往后时间段的tip肯定就基本稳定位置
data7 = ori_data7(: , indice7);

data = [data1, data2, data3, data4, data5, data6, data7];
data(1, :) = []; 

col_num = size(data, 2);
rand_index = randperm(col_num); %随机排列向量，接下来用来划分训练集和验证集
select_num = floor(col_num*0.8);
idx = rand_index(1:select_num);
train_data = data(:,idx);
idx = rand_index(select_num+1:end);
validation_data = data(:,idx);



%构建模型%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
motion_data = train_data(8:9, : ); %start_point的数据会干扰训练准确
magnet_data = train_data([1, 2, 15], : );
start_motion_data = train_data(6:7, : );
inputdata = motion_data - start_motion_data; %建立相对坐标系，可能会准确一些

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
net.trainParam.max_fail = 210;  % 验证失败次数超过 ... 次时停止

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
validation_input  = validation_data(8:9, : ) - validation_data(6:7, : ); 
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
save('trained_net_2inputs_2outputs.mat', 'net');






