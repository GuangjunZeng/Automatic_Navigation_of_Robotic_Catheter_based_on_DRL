clear;

% 读取excel数据
image = xlsread('C:\Users\admin\Desktop\Labview2018\测试视频和图片\image5.xlsx');

% 创建二值图像
binaryImage = image == 0;

% 使用细化算法找到形状的中心线
thinImage = bwmorph(binaryImage, 'thin', Inf);

% 找到中心线上的所有点
[y, x] = find(thinImage);

% 初始化端点跟踪数组
endPoints = zeros(2,2);

% CTRL: 端点找到的计数
ctrl = 0;

% 检查每个中心线上的点
for k = 1:length(x)
    i = x(k);
    j = y(k);
    % 检查其周围8个格子
    neighbors = thinImage(max(j-1, 1):min(j+1, size(thinImage,1)), max(i-1, 1):min(i+1, size(thinImage,2)));
    % 检查邻居中1的个数
    if sum(neighbors(:)) < 3 % 如果只有一个或两个邻居，那么当前格子是端点
        ctrl = ctrl + 1;
        endPoints(ctrl, :) = [i j];
        if ctrl>=2
            break;
        end
    end
end

% 两个端点离得足够远，选择一个端点作为起点
if ctrl==2 && sqrt(sum((endPoints(1,:) - endPoints(2,:)) .^2)) > 2
    % 选择一个端点作为起点
    x_start = endPoints(1,1);
    y_start = endPoints(1,2);

    Path(1,:) = [x_start, y_start];
    % 从x和y列表中删除已经添加到路径的点
    index = x==Path(1,1) & y==Path(1,2);
    x(index) = [];
    y(index) = [];

    % 当没有可选点时，路径构建中止
    while ~isempty(x)
        Distances = (x-Path(end,1)).^2 + (y-Path(end,2)).^2; % 计算所有点到当前路径末端的距离
        [minDist, nextPointIdx] = min(Distances); % 找到最近的点
        if minDist > 2 % 如果最近的点的距离大于2，那么这可能是一个段点，就结束路径的构建
            break;
        end
    
        nextPoint = [x(nextPointIdx), y(nextPointIdx)]; %选取下一步
        Path = [Path; nextPoint]; %在Path中加入新的节点
    
        x(nextPointIdx) = []; % 从x列表中删除这个节点
        y(nextPointIdx) = []; % 从y列表中删除这个节点
    end


    % 打印起点和终点
    disp(['起点: ', num2str(Path(1,:))]);
    disp(['终点: ', num2str(Path(end,:))]);

    % 输出结束的路径的点数（可以用于调试或确定优化的方向）
    disp(['路径点数: ', num2str(size(Path, 1))]);

    % 绘制原始的二值图像
    figure;
    imshow(~binaryImage);
    
    %留出部分空间
    xrange = xlim; % 获取当前x轴范围
    yrange = ylim; % 获取当前y轴范围
    
    extend_range = 100; % 延伸的范围，这可以根据你的需要进行设置
    
    xlim([xrange(1)-extend_range, xrange(2)+extend_range]);
    ylim([yrange(1)-extend_range, yrange(2)+extend_range]);
    
    hold on;
    
    % 绘制中心线路径
    plot(Path(:,1), Path(:,2), 'r-', 'LineWidth', 2); % 绘制所有点

    % 在起点和终点绘制一个圈
    plot(Path(1,1), Path(1,2), 'go', 'MarkerSize',10, 'MarkerFaceColor', 'g');
    plot(Path(end,1), Path(end,2), 'bo', 'MarkerSize',10, 'MarkerFaceColor', 'b');

    % 在图像上标注起点和终点
    text(Path(1,1), Path(1,2), '起点', 'Color', 'b', 'FontSize', 14);
    text(Path(end,1), Path(end,2), '终点', 'Color', 'b', 'FontSize', 14);

    title('二值图像和其中心线');
    xlabel('X');
    ylabel('Y');

    hold off;
end


% 基于选择的点的四周的邻居情况，判断该点是否可以用于扩展路径
function extendable = isPathExtendable(currentPoint, nextPoints, thinImage)
    extendable = false(size(nextPoints, 1), 1); % 初始化extendable数组
    for k = 1:size(nextPoints, 1)
        i = nextPoints(k, 1);
        j = nextPoints(k, 2);
        % 检查其周围8个格子
        neighbors = thinImage(max(j-1, 1):min(j+1, size(thinImage,1)), max(i-1, 1):min(i+1, size(thinImage,2)));
        % 如果四周有超过1个邻居（包括自己），那么这个点就可以用于扩展路径
        if sum(neighbors(:)) > 2
            extendable(k) = true;
        end
    end
end
