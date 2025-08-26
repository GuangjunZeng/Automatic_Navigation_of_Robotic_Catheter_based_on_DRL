% 定义四个角点的坐标
start_point = [1400-1000, 900-900]; % 右上角
bottom_right = [1400-1000, 1400-900]; % 右下角
bottom_left = [600-1000, 1400-900]; % 左下角
top_left = [600-1000, 900-900]; % 左上角

% 将四个点按顺时针顺序排列
square_points = [start_point; bottom_right; bottom_left; top_left; start_point];

% 存储坐标点的矩阵
trajectory = [];

% 生成轨迹
for i = 1:4
    % 计算两个连续点之间的差值
    x_diff = square_points(i+1, 1) - square_points(i, 1);
    y_diff = square_points(i+1, 2) - square_points(i, 2);
    
    % 计算步长，假设每次移动10个单位
    steps = max(abs(x_diff), abs(y_diff)) / 10;
    
    % 计算每一步的增量
    x_step = x_diff / steps;
    y_step = y_diff / steps;
    
    % 生成每一小步的坐标点
    for j = 0:steps
        new_point = square_points(i, :) + [x_step * j, y_step * j];
        trajectory = [trajectory; new_point];
    end
end

save('E:\capstone2025\small_myRIO_Helmholtz PID controller\trajectory_data.mat', 'trajectory');


% 绘制轨迹
figure;
plot(trajectory(:,1), trajectory(:,2), '-o');
xlabel('X');
ylabel('Y');
title('Square Trajectory Tracking');
axis equal;
grid on;
