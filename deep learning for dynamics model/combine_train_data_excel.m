folderPath = 'C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\'; % 设置文件夹路径
% 假设五个表格的文件名分别为 'file1.xlsx', 'file2.xlsx', ..., 'file5.xlsx'
files = {'train1.xlsx', 'train2.xlsx', 'train3.xlsx', 'train4.xlsx', 'train5.xlsx'};

% 初始化一个空表格，用于存储合并后的数据
combinedData = [];

% 遍历每个文件，读取数据并将其按行合并
for i = 1:length(files)
    % 拼接完整的文件路径
    filePath = fullfile(folderPath, files{i});
    % 读取当前 Excel 文件
    currentTable = readtable(filePath);
    
    % 合并数据（按行）
    combinedData = [combinedData; currentTable];
end

% 输出合并后的表格到一个新的 Excel 文件
writetable(combinedData, 'combinedData.xlsx');


