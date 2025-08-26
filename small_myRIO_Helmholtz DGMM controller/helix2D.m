% 设置螺旋参数
radius_start = 0;        % 起始半径
radius_end = 200;       % 结束半径
z_start = 0;             % 起始高度
z_end = 500;             % 结束高度
turns = 2;              % 螺旋圈数
points_per_turn = 100;   % 每圈的点数

% 计算螺旋线的每一圈的角度和半径的变化
theta = linspace(0, 2 * pi * turns, points_per_turn * turns);
radii = linspace(radius_start, radius_end, points_per_turn * turns);
z = linspace(z_start, z_end, points_per_turn * turns);

% 计算螺旋线的x, y坐标
x = radii .* cos(theta);
y = radii .* sin(theta);

% 将整个螺旋线向y轴正方向移动200个单位
y = y + 250;


% 存储轨迹
trajectory = [x', y'];

% 保存轨迹
save('F:\capstone2025\small_myRIO_Helmholtz PID controller\trajectory_helix.mat', 'trajectory');

% 绘制螺旋轨迹
figure;
plot(trajectory(:, 1), trajectory(:, 2), '-o');
xlabel('X');
ylabel('Y');
title('Helical Trajectory');
axis equal;
grid on;

