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
%data = data1;
data(1, :) = []; 


col_num = size(data, 2);
select_num = floor(col_num * 0.8);
%rand_index = randperm(col_num, col_num - select_num); % 随机抽取20%的索引作为验证集
%validation_data = data(:, rand_index);
%train_data = data(:, setdiff(1:col_num, rand_index)); % 剩余的作为训练集


motion_data = data(8:9, : ); %start_point的数据会干扰训练准确
magnet_data = data([1, 2, 15], : );
start_motion_data = data(6:7, : );

now_point_data = motion_data - start_motion_data; %建立相对坐标系，
now_point_data_without_last_col = now_point_data(:, 1:end-1);
previous_point_data = [ [0;0], now_point_data_without_last_col];

delta_data = now_point_data - previous_point_data;

dr = magnet_data(2, : );
pr = magnet_data(3, : );
u_data = magnet_data(2:3, : );

X_train = [now_point_data; u_data];
% 中心化 delta_data
delta_data_mean = mean(delta_data);
delta_data_centered = delta_data - delta_data_mean;

Y_train = delta_data_centered;

X_train = X_train';
Y_train = Y_train';

max_dr = max(dr);
max_pr = max(pr);
%disp(['train_data_input_max_dr = ' num2str(max_dr)]);
%disp(['train_data_input_max_pr = ' num2str(max_pr)]);


%训练高斯过程回归模型%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m = 2; %m是输出的维度
% 训练每个输出维度的GPR模型
gp_models = cell(m, 1);

%?????分布的均值（期望值）为 0, 数据要进行中心化？！会对后续过程有什么影响吗？
for d = 1:m
    gp_models{d} = fitrgp(X_train, Y_train(:,d), ...
        'KernelFunction','squaredexponential', ...
        'OptimizeHyperparameters', 'all', ...
        'HyperparameterOptimizationOptions', struct('MaxObjectiveEvaluations', 50));
end


%保存训练的网络到本地一个新的文件中%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save('trained_net_4inputs_2outputs.mat', 'gp_models');






