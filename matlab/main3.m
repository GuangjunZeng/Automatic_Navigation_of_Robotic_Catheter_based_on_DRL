% 读取excel数据
data = xlsread('C:\Users\admin\Desktop\Labview2018\测试视频和图片\image2.xlsx');

% 创建二值图像
binaryImage = data == 0;

% 使用细化算法找到形状的中心线
thinImage = bwmorph(binaryImage, 'thin', Inf);

figure, imshow(thinImage);
title('Thinned Image');

% 找到中心线上的所有点
[y, x] = find(thinImage == 1);

%选择初始点（例如第一个点）
Path(1,:) = [x(1), y(1)];
disp(['初始点: ', num2str(x(1)), ', ', num2str(y(1))]);
disp(['终端点: ', num2str(x(end)), ', ', num2str(y(end))]);

%从x和y列表中删除已经添加到路径的点
x(1) = [];
y(1) = [];

threshold = 1; % 设置阈值

i = 2; % 设置一个计数器
while ~isempty(x) && ~isempty(y)
  
    distances = sqrt((x-Path(i-1,1)).^2 + (y-Path(i-1,2)).^2);

    [~, index] = min(distances);

    if min(distances) > threshold
        break;
    end
    
    % Additional check to ensure index is not out of bounds
    if index <= numel(x) && index <= numel(y)
        if thinImage(y(index), x(index)) == 1 % Make sure point is white
            Path(i,:) = [x(index), y(index)];

            x(index) = [];
            y(index) = [];
        
            i = i + 1;
        else
            % If the point is not white, remove it from the list
            x(index) = [];
            y(index) = [];
        end
    else
        break; % If index is out of bounds, break the loop
    end
end

% 打印起点和终点
disp(['起点: ', num2str(Path(1,:))]);
disp(['终点: ', num2str(Path(end,:))]);


% 绘制原始的二值图像
figure;
imshow(~binaryImage); % 使用~操作符来反转图像
hold on;

% 绘制中心线路径
plot(Path(:,1), Path(:,2), 'r-', 'LineWidth', 2); % 绘制所有点

% 在起点和终点绘制一个圈
plot(Path(1,1), Path(1,2), 'go', 'MarkerSize',10, 'MarkerFaceColor', 'g');
plot(Path(end,1), Path(end,2), 'bo', 'MarkerSize',10, 'MarkerFaceColor', 'b');

% 在图像上标注起点和终点
text(Path(1,1), Path(1,2), '起点', 'Color', 'b', 'FontSize', 14);
text(Path(end,1), Path(end,2), '终点', 'Color', 'b', 'FontSize', 14);

start_point_x = num2str(Path(1,:));
title('二值图像和其中心线');
xlabel('X');
ylabel('Y');

% 显示坐标轴刻度
xticks(1:size(data, 2));
yticks(1:size(data, 1));

hold off;
