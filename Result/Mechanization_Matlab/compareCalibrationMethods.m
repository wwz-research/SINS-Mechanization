function compareCalibrationMethods()
% 对比多个校准方法的误差时序图并计算误差下降百分比
% 功能：
%   1. 读取Mechanization_Cali、Mechanization_accCali、Mechanization_gyrCali、
%      Mechanization_IsZero的误差数据
%   2. 在同一子图中绘制对应误差的对比曲线
%   3. 计算Mechanization_Cali、Mechanization_accCali、Mechanization_gyrCali
%      误差的Mean、RMSE相较于Mechanization_IsZero误差下降多少

    clear;
    close all;
    clc;

    % 设置数据文件路径
    dataPath_IsZero = 'Mechanization_IsZero';
    dataPath_Cali = 'Mechanization_Cali';
    dataPath_accCali = 'Mechanization_accCali';
    dataPath_gyrCali = 'Mechanization_gyrCali';
    
    resultDiffFile_IsZero = fullfile(dataPath_IsZero, 'result_diff.txt');
    resultDiffFile_Cali = fullfile(dataPath_Cali, 'result_diff.txt');
    resultDiffFile_accCali = fullfile(dataPath_accCali, 'result_diff.txt');
    resultDiffFile_gyrCali = fullfile(dataPath_gyrCali, 'result_diff.txt');

    % 读取数据
    fprintf('正在读取数据文件...\n');
    result_diff_IsZero = readDataDiff(resultDiffFile_IsZero);
    result_diff_Cali = readDataDiff(resultDiffFile_Cali);
    result_diff_accCali = readDataDiff(resultDiffFile_accCali);
    result_diff_gyrCali = readDataDiff(resultDiffFile_gyrCali);

    % 检查数据长度是否一致，使用最短长度
    lengths = [height(result_diff_IsZero), height(result_diff_Cali), ...
               height(result_diff_accCali), height(result_diff_gyrCali)];
    minLen = min(lengths);
    if length(unique(lengths)) > 1
        warning('数据长度不一致，将使用最短长度 %d', minLen);
    end
    
    result_diff_IsZero = result_diff_IsZero(1:minLen, :);
    result_diff_Cali = result_diff_Cali(1:minLen, :);
    result_diff_accCali = result_diff_accCali(1:minLen, :);
    result_diff_gyrCali = result_diff_gyrCali(1:minLen, :);

    % 提取时间序列（GPS周秒）
    time = result_diff_IsZero.GPSTime;

    fprintf('数据读取完成，共 %d 个数据点\n', length(time));

    % 处理航向角误差：解决取值范围突变问题
    % 大于300的减去360，小于-300的加上360
    result_diff_IsZero = processYawError(result_diff_IsZero);
    result_diff_Cali = processYawError(result_diff_Cali);
    result_diff_accCali = processYawError(result_diff_accCali);
    result_diff_gyrCali = processYawError(result_diff_gyrCali);

    % 创建对比图
    figure(6);
    clf;
    
    % ========== 第一行：ENU坐标系误差 ==========
    % dE误差
    subplot(3,3,1);
    plot(time, result_diff_IsZero.dE, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_Cali.dE, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Cali');
    plot(time, result_diff_accCali.dE, 'g-', 'LineWidth', 1.5, 'DisplayName', 'accCali');
    plot(time, result_diff_gyrCali.dE, 'm-', 'LineWidth', 1.5, 'DisplayName', 'gyrCali');
    xlabel('GPS周秒');
    ylabel('dE (m)');
    title('E方向误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % dN误差
    subplot(3,3,2);
    plot(time, result_diff_IsZero.dN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_Cali.dN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Cali');
    plot(time, result_diff_accCali.dN, 'g-', 'LineWidth', 1.5, 'DisplayName', 'accCali');
    plot(time, result_diff_gyrCali.dN, 'm-', 'LineWidth', 1.5, 'DisplayName', 'gyrCali');
    xlabel('GPS周秒');
    ylabel('dN (m)');
    title('N方向误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % dU误差
    subplot(3,3,3);
    plot(time, result_diff_IsZero.dU, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_Cali.dU, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Cali');
    plot(time, result_diff_accCali.dU, 'g-', 'LineWidth', 1.5, 'DisplayName', 'accCali');
    plot(time, result_diff_gyrCali.dU, 'm-', 'LineWidth', 1.5, 'DisplayName', 'gyrCali');
    xlabel('GPS周秒');
    ylabel('dU (m)');
    title('U方向误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % ========== 第二行：速度误差 ==========
    % 北向速度误差
    subplot(3,3,4);
    plot(time, result_diff_IsZero.Velocity_N, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_Cali.Velocity_N, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Cali');
    plot(time, result_diff_accCali.Velocity_N, 'g-', 'LineWidth', 1.5, 'DisplayName', 'accCali');
    plot(time, result_diff_gyrCali.Velocity_N, 'm-', 'LineWidth', 1.5, 'DisplayName', 'gyrCali');
    xlabel('GPS周秒');
    ylabel('北向速度误差 (m/s)');
    title('北向速度误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 东向速度误差
    subplot(3,3,5);
    plot(time, result_diff_IsZero.Velocity_E, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_Cali.Velocity_E, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Cali');
    plot(time, result_diff_accCali.Velocity_E, 'g-', 'LineWidth', 1.5, 'DisplayName', 'accCali');
    plot(time, result_diff_gyrCali.Velocity_E, 'm-', 'LineWidth', 1.5, 'DisplayName', 'gyrCali');
    xlabel('GPS周秒');
    ylabel('东向速度误差 (m/s)');
    title('东向速度误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 垂向速度误差
    subplot(3,3,6);
    plot(time, result_diff_IsZero.Velocity_D, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_Cali.Velocity_D, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Cali');
    plot(time, result_diff_accCali.Velocity_D, 'g-', 'LineWidth', 1.5, 'DisplayName', 'accCali');
    plot(time, result_diff_gyrCali.Velocity_D, 'm-', 'LineWidth', 1.5, 'DisplayName', 'gyrCali');
    xlabel('GPS周秒');
    ylabel('垂向速度误差 (m/s)');
    title('垂向速度误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % ========== 第三行：姿态误差 ==========
    % 横滚角误差
    subplot(3,3,7);
    plot(time, result_diff_IsZero.Roll, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_Cali.Roll, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Cali');
    plot(time, result_diff_accCali.Roll, 'g-', 'LineWidth', 1.5, 'DisplayName', 'accCali');
    plot(time, result_diff_gyrCali.Roll, 'm-', 'LineWidth', 1.5, 'DisplayName', 'gyrCali');
    xlabel('GPS周秒');
    ylabel('横滚角误差 (deg)');
    title('横滚角误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 俯仰角误差
    subplot(3,3,8);
    plot(time, result_diff_IsZero.Pitch, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_Cali.Pitch, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Cali');
    plot(time, result_diff_accCali.Pitch, 'g-', 'LineWidth', 1.5, 'DisplayName', 'accCali');
    plot(time, result_diff_gyrCali.Pitch, 'm-', 'LineWidth', 1.5, 'DisplayName', 'gyrCali');
    xlabel('GPS周秒');
    ylabel('俯仰角误差 (deg)');
    title('俯仰角误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 航向角误差
    subplot(3,3,9);
    plot(time, result_diff_IsZero.Yaw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_Cali.Yaw, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Cali');
    plot(time, result_diff_accCali.Yaw, 'g-', 'LineWidth', 1.5, 'DisplayName', 'accCali');
    plot(time, result_diff_gyrCali.Yaw, 'm-', 'LineWidth', 1.5, 'DisplayName', 'gyrCali');
    xlabel('GPS周秒');
    ylabel('航向角误差 (deg)');
    title('航向角误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    sgtitle('误差时序对比图 (IsZero vs Cali vs accCali vs gyrCali)', ...
            'FontSize', 14, 'FontWeight', 'bold');

    % ========== 计算并输出统计指标和下降百分比 ==========
    fprintf('\n========== 误差统计指标对比（相对于IsZero） ==========\n');
    
    % 计算并输出各个误差指标的统计信息
    calculateAndPrintReduction(result_diff_IsZero, result_diff_Cali, 'Cali', '【ENU坐标系误差】', ...
        'dE', 'dN', 'dU', 'm');
    calculateAndPrintReduction(result_diff_IsZero, result_diff_accCali, 'accCali', '', ...
        'dE', 'dN', 'dU', 'm');
    calculateAndPrintReduction(result_diff_IsZero, result_diff_gyrCali, 'gyrCali', '', ...
        'dE', 'dN', 'dU', 'm');
    
    calculateAndPrintReduction(result_diff_IsZero, result_diff_Cali, 'Cali', '\n【速度误差】', ...
        'Velocity_N', 'Velocity_E', 'Velocity_D', 'm/s', ...
        '北向速度', '东向速度', '垂向速度');
    calculateAndPrintReduction(result_diff_IsZero, result_diff_accCali, 'accCali', '', ...
        'Velocity_N', 'Velocity_E', 'Velocity_D', 'm/s', ...
        '北向速度', '东向速度', '垂向速度');
    calculateAndPrintReduction(result_diff_IsZero, result_diff_gyrCali, 'gyrCali', '', ...
        'Velocity_N', 'Velocity_E', 'Velocity_D', 'm/s', ...
        '北向速度', '东向速度', '垂向速度');
    
    calculateAndPrintReduction(result_diff_IsZero, result_diff_Cali, 'Cali', '\n【姿态误差】', ...
        'Roll', 'Pitch', 'Yaw', 'deg', ...
        '横滚角', '俯仰角', '航向角');
    calculateAndPrintReduction(result_diff_IsZero, result_diff_accCali, 'accCali', '', ...
        'Roll', 'Pitch', 'Yaw', 'deg', ...
        '横滚角', '俯仰角', '航向角');
    calculateAndPrintReduction(result_diff_IsZero, result_diff_gyrCali, 'gyrCali', '', ...
        'Roll', 'Pitch', 'Yaw', 'deg', ...
        '横滚角', '俯仰角', '航向角');
    
    fprintf('==================================\n\n');
    
    fprintf('对比图表绘制完成！\n');
end

% 辅助函数：处理航向角误差
function result_diff = processYawError(result_diff)
    if ismember('Yaw', result_diff.Properties.VariableNames)
        yaw_error = result_diff.Yaw;
        yaw_error(yaw_error > 300) = yaw_error(yaw_error > 300) - 360;
        yaw_error(yaw_error < -300) = yaw_error(yaw_error < -300) + 360;
        result_diff.Yaw = yaw_error;
    end
end

% 辅助函数：计算并打印下降百分比
function calculateAndPrintReduction(result_original, result_method, methodName, sectionTitle, ...
                                    field1, field2, field3, unit, name1, name2, name3)
    % 如果未提供名称，使用字段名作为名称
    if nargin < 9
        name1 = field1;
        name2 = field2;
        name3 = field3;
    end
    
    if ~isempty(sectionTitle)
        fprintf('%s\n', sectionTitle);
    end
    
    % 计算第一个字段
    original_mean = mean(result_original.(field1));
    original_rmse = sqrt(mean(result_original.(field1).^2));
    method_mean = mean(result_method.(field1));
    method_rmse = sqrt(mean(result_method.(field1).^2));
    mean_reduction = calculateReduction(original_mean, method_mean);
    rmse_reduction = calculateReduction(original_rmse, method_rmse);
    fprintf('%s (%s):\n', name1, methodName);
    fprintf('  IsZero:  Mean = %12.6f %s,  RMSE = %12.6f %s\n', original_mean, unit, original_rmse, unit);
    fprintf('  %s:    Mean = %12.6f %s,  RMSE = %12.6f %s\n', methodName, method_mean, unit, method_rmse, unit);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', mean_reduction, rmse_reduction);
    
    % 计算第二个字段
    original_mean = mean(result_original.(field2));
    original_rmse = sqrt(mean(result_original.(field2).^2));
    method_mean = mean(result_method.(field2));
    method_rmse = sqrt(mean(result_method.(field2).^2));
    mean_reduction = calculateReduction(original_mean, method_mean);
    rmse_reduction = calculateReduction(original_rmse, method_rmse);
    fprintf('%s (%s):\n', name2, methodName);
    fprintf('  IsZero:  Mean = %12.6f %s,  RMSE = %12.6f %s\n', original_mean, unit, original_rmse, unit);
    fprintf('  %s:    Mean = %12.6f %s,  RMSE = %12.6f %s\n', methodName, method_mean, unit, method_rmse, unit);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', mean_reduction, rmse_reduction);
    
    % 计算第三个字段
    original_mean = mean(result_original.(field3));
    original_rmse = sqrt(mean(result_original.(field3).^2));
    method_mean = mean(result_method.(field3));
    method_rmse = sqrt(mean(result_method.(field3).^2));
    mean_reduction = calculateReduction(original_mean, method_mean);
    rmse_reduction = calculateReduction(original_rmse, method_rmse);
    fprintf('%s (%s):\n', name3, methodName);
    fprintf('  IsZero:  Mean = %12.6f %s,  RMSE = %12.6f %s\n', original_mean, unit, original_rmse, unit);
    fprintf('  %s:    Mean = %12.6f %s,  RMSE = %12.6f %s\n', methodName, method_mean, unit, method_rmse, unit);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', mean_reduction, rmse_reduction);
end

% 辅助函数：计算下降百分比
function reduction = calculateReduction(original, method)
    % 计算下降百分比：(original - method) / |original| * 100
    % 对于Mean使用绝对值，对于RMSE（总是正数）abs(original)就是original本身
    if abs(original) > 1e-10
        reduction = (original - method) / abs(original) * 100;
    else
        reduction = 0;
    end
end

