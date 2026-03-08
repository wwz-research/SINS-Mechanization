% 主函数：惯导解算结果可视化程序
% 功能：读取数据文件并绘制对比图和误差图

clear;
close all;
clc;

% 设置数据文件路径
dataPath = 'Mechanization';
% dataPath = 'MechanizationExample';
% dataPath = 'Mechanization_original';
% dataPath = 'Mechanization_IsZero';
% dataPath = 'Mechanization_vchange';
% dataPath = 'Mechanization_Cali';
% dataPath = 'Mechanization_accCali';
% dataPath = 'Mechanization_gyrCali';
resultFile = fullfile(dataPath, 'result.txt');
resultRefFile = fullfile(dataPath, 'result_ref.txt');
resultDiffFile = fullfile(dataPath, 'result_diff.txt');
resultDENUFile = fullfile(dataPath, 'result_denu.txt');

% 读取数据
fprintf('正在读取数据文件...\n');
result = readData(resultFile);
result_ref = readData(resultRefFile);
result_diff = readDataDiff(resultDiffFile);  % 使用readDataDiff读取包含dE、dN、dU的数据
result_denu = readDataDENU(resultDENUFile);  % 读取轨迹数据

% 检查数据长度是否一致
if height(result) ~= height(result_ref) || height(result) ~= height(result_diff)
    warning('数据长度不一致，将使用最短长度');
    minLen = min([height(result), height(result_ref), height(result_diff)]);
    result = result(1:minLen, :);
    result_ref = result_ref(1:minLen, :);
    result_diff = result_diff(1:minLen, :);
end

% 提取时间序列（GPS周秒）
time = result.GPSTime;

fprintf('数据读取完成，共 %d 个数据点\n', length(time));

% 绘制对比图（figure1）
fprintf('正在绘制对比图...\n');
plotComparison(result, result_ref, time);

% 绘制轨迹对比图（figure2和figure3）
fprintf('正在绘制轨迹对比图...\n');
plotTrajectory(result_denu);

% 绘制所有误差图并计算统计指标（figure4）
fprintf('正在绘制误差图并计算统计指标...\n');
plotAllError(result_diff, time);

fprintf('所有图表绘制完成！\n');

