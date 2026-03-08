function compareError()
% 对比两个数据文件夹的误差时序图并计算误差下降百分比
% 功能：
%   1. 读取Mechanization_original和Mechanization_IsZero的误差数据
%   2. 在同一子图中绘制对应误差的对比曲线
%   3. 计算Mechanization_IsZero误差的Mean、RMSE相较于Mechanization_original误差下降多少

    clear;
    close all;
    clc;

    % 设置数据文件路径
    dataPath_original = 'Mechanization_original';
    dataPath_IsZero = 'Mechanization_IsZero';
    
    resultDiffFile_original = fullfile(dataPath_original, 'result_diff.txt');
    resultDiffFile_IsZero = fullfile(dataPath_IsZero, 'result_diff.txt');

    % 读取数据
    fprintf('正在读取数据文件...\n');
    result_diff_original = readDataDiff(resultDiffFile_original);
    result_diff_IsZero = readDataDiff(resultDiffFile_IsZero);

    % 检查数据长度是否一致
    if height(result_diff_original) ~= height(result_diff_IsZero)
        warning('数据长度不一致，将使用最短长度');
        minLen = min([height(result_diff_original), height(result_diff_IsZero)]);
        result_diff_original = result_diff_original(1:minLen, :);
        result_diff_IsZero = result_diff_IsZero(1:minLen, :);
    end

    % 提取时间序列（GPS周秒）
    time = result_diff_original.GPSTime;

    fprintf('数据读取完成，共 %d 个数据点\n', length(time));

    % 处理航向角误差：解决取值范围突变问题
    % 大于300的减去360，小于-300的加上360
    if ismember('Yaw', result_diff_original.Properties.VariableNames)
        yaw_error_original = result_diff_original.Yaw;
        yaw_error_original(yaw_error_original > 300) = yaw_error_original(yaw_error_original > 300) - 360;
        yaw_error_original(yaw_error_original < -300) = yaw_error_original(yaw_error_original < -300) + 360;
        result_diff_original.Yaw = yaw_error_original;
    end
    
    if ismember('Yaw', result_diff_IsZero.Properties.VariableNames)
        yaw_error_IsZero = result_diff_IsZero.Yaw;
        yaw_error_IsZero(yaw_error_IsZero > 300) = yaw_error_IsZero(yaw_error_IsZero > 300) - 360;
        yaw_error_IsZero(yaw_error_IsZero < -300) = yaw_error_IsZero(yaw_error_IsZero < -300) + 360;
        result_diff_IsZero.Yaw = yaw_error_IsZero;
    end

    % 创建对比图
    figure(5);
    clf;
    
    % ========== 第一行：ENU坐标系误差 ==========
    % dE误差
    subplot(3,3,1);
    plot(time, result_diff_original.dE, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(time, result_diff_IsZero.dE, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    xlabel('GPS周秒');
    ylabel('dE (m)');
    title('E方向误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % dN误差
    subplot(3,3,2);
    plot(time, result_diff_original.dN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(time, result_diff_IsZero.dN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    xlabel('GPS周秒');
    ylabel('dN (m)');
    title('N方向误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % dU误差
    subplot(3,3,3);
    plot(time, result_diff_original.dU, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(time, result_diff_IsZero.dU, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    xlabel('GPS周秒');
    ylabel('dU (m)');
    title('U方向误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % ========== 第二行：速度误差 ==========
    % 北向速度误差
    subplot(3,3,4);
    plot(time, result_diff_original.Velocity_N, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(time, result_diff_IsZero.Velocity_N, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    xlabel('GPS周秒');
    ylabel('北向速度误差 (m/s)');
    title('北向速度误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 东向速度误差
    subplot(3,3,5);
    plot(time, result_diff_original.Velocity_E, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(time, result_diff_IsZero.Velocity_E, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    xlabel('GPS周秒');
    ylabel('东向速度误差 (m/s)');
    title('东向速度误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 垂向速度误差
    subplot(3,3,6);
    plot(time, result_diff_original.Velocity_D, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(time, result_diff_IsZero.Velocity_D, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    xlabel('GPS周秒');
    ylabel('垂向速度误差 (m/s)');
    title('垂向速度误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % ========== 第三行：姿态误差 ==========
    % 横滚角误差
    subplot(3,3,7);
    plot(time, result_diff_original.Roll, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(time, result_diff_IsZero.Roll, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    xlabel('GPS周秒');
    ylabel('横滚角误差 (deg)');
    title('横滚角误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 俯仰角误差
    subplot(3,3,8);
    plot(time, result_diff_original.Pitch, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(time, result_diff_IsZero.Pitch, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    xlabel('GPS周秒');
    ylabel('俯仰角误差 (deg)');
    title('俯仰角误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 航向角误差
    subplot(3,3,9);
    plot(time, result_diff_original.Yaw, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(time, result_diff_IsZero.Yaw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IsZero');
    xlabel('GPS周秒');
    ylabel('航向角误差 (deg)');
    title('航向角误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    sgtitle('误差时序对比图 (Original vs IsZero)', 'FontSize', 14, 'FontWeight', 'bold');

    % ========== 计算并输出统计指标和下降百分比 ==========
    fprintf('\n========== 误差统计指标对比 ==========\n');
    
    % ENU坐标系误差统计
    fprintf('\n【ENU坐标系误差】\n');
    
    % dE
    dE_mean_original = mean(result_diff_original.dE);
    dE_rmse_original = sqrt(mean(result_diff_original.dE.^2));
    dE_mean_IsZero = mean(result_diff_IsZero.dE);
    dE_rmse_IsZero = sqrt(mean(result_diff_IsZero.dE.^2));
    if abs(dE_mean_original) > 1e-10
        dE_mean_reduction = (dE_mean_original - dE_mean_IsZero) / abs(dE_mean_original) * 100;
    else
        dE_mean_reduction = 0;
    end
    if dE_rmse_original > 1e-10
        dE_rmse_reduction = (dE_rmse_original - dE_rmse_IsZero) / dE_rmse_original * 100;
    else
        dE_rmse_reduction = 0;
    end
    fprintf('dE:\n');
    fprintf('  Original:  Mean = %12.6f m,  RMSE = %12.6f m\n', dE_mean_original, dE_rmse_original);
    fprintf('  IsZero:    Mean = %12.6f m,  RMSE = %12.6f m\n', dE_mean_IsZero, dE_rmse_IsZero);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', dE_mean_reduction, dE_rmse_reduction);
    
    % dN
    dN_mean_original = mean(result_diff_original.dN);
    dN_rmse_original = sqrt(mean(result_diff_original.dN.^2));
    dN_mean_IsZero = mean(result_diff_IsZero.dN);
    dN_rmse_IsZero = sqrt(mean(result_diff_IsZero.dN.^2));
    if abs(dN_mean_original) > 1e-10
        dN_mean_reduction = (dN_mean_original - dN_mean_IsZero) / abs(dN_mean_original) * 100;
    else
        dN_mean_reduction = 0;
    end
    if dN_rmse_original > 1e-10
        dN_rmse_reduction = (dN_rmse_original - dN_rmse_IsZero) / dN_rmse_original * 100;
    else
        dN_rmse_reduction = 0;
    end
    fprintf('dN:\n');
    fprintf('  Original:  Mean = %12.6f m,  RMSE = %12.6f m\n', dN_mean_original, dN_rmse_original);
    fprintf('  IsZero:    Mean = %12.6f m,  RMSE = %12.6f m\n', dN_mean_IsZero, dN_rmse_IsZero);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', dN_mean_reduction, dN_rmse_reduction);
    
    % dU
    dU_mean_original = mean(result_diff_original.dU);
    dU_rmse_original = sqrt(mean(result_diff_original.dU.^2));
    dU_mean_IsZero = mean(result_diff_IsZero.dU);
    dU_rmse_IsZero = sqrt(mean(result_diff_IsZero.dU.^2));
    if abs(dU_mean_original) > 1e-10
        dU_mean_reduction = (dU_mean_original - dU_mean_IsZero) / abs(dU_mean_original) * 100;
    else
        dU_mean_reduction = 0;
    end
    if dU_rmse_original > 1e-10
        dU_rmse_reduction = (dU_rmse_original - dU_rmse_IsZero) / dU_rmse_original * 100;
    else
        dU_rmse_reduction = 0;
    end
    fprintf('dU:\n');
    fprintf('  Original:  Mean = %12.6f m,  RMSE = %12.6f m\n', dU_mean_original, dU_rmse_original);
    fprintf('  IsZero:    Mean = %12.6f m,  RMSE = %12.6f m\n', dU_mean_IsZero, dU_rmse_IsZero);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', dU_mean_reduction, dU_rmse_reduction);
    
    % 速度误差统计
    fprintf('\n【速度误差】\n');
    
    % 北向速度
    vN_mean_original = mean(result_diff_original.Velocity_N);
    vN_rmse_original = sqrt(mean(result_diff_original.Velocity_N.^2));
    vN_mean_IsZero = mean(result_diff_IsZero.Velocity_N);
    vN_rmse_IsZero = sqrt(mean(result_diff_IsZero.Velocity_N.^2));
    if abs(vN_mean_original) > 1e-10
        vN_mean_reduction = (vN_mean_original - vN_mean_IsZero) / abs(vN_mean_original) * 100;
    else
        vN_mean_reduction = 0;
    end
    if vN_rmse_original > 1e-10
        vN_rmse_reduction = (vN_rmse_original - vN_rmse_IsZero) / vN_rmse_original * 100;
    else
        vN_rmse_reduction = 0;
    end
    fprintf('北向速度:\n');
    fprintf('  Original:  Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vN_mean_original, vN_rmse_original);
    fprintf('  IsZero:    Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vN_mean_IsZero, vN_rmse_IsZero);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', vN_mean_reduction, vN_rmse_reduction);
    
    % 东向速度
    vE_mean_original = mean(result_diff_original.Velocity_E);
    vE_rmse_original = sqrt(mean(result_diff_original.Velocity_E.^2));
    vE_mean_IsZero = mean(result_diff_IsZero.Velocity_E);
    vE_rmse_IsZero = sqrt(mean(result_diff_IsZero.Velocity_E.^2));
    if abs(vE_mean_original) > 1e-10
        vE_mean_reduction = (vE_mean_original - vE_mean_IsZero) / abs(vE_mean_original) * 100;
    else
        vE_mean_reduction = 0;
    end
    if vE_rmse_original > 1e-10
        vE_rmse_reduction = (vE_rmse_original - vE_rmse_IsZero) / vE_rmse_original * 100;
    else
        vE_rmse_reduction = 0;
    end
    fprintf('东向速度:\n');
    fprintf('  Original:  Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vE_mean_original, vE_rmse_original);
    fprintf('  IsZero:    Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vE_mean_IsZero, vE_rmse_IsZero);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', vE_mean_reduction, vE_rmse_reduction);
    
    % 垂向速度
    vD_mean_original = mean(result_diff_original.Velocity_D);
    vD_rmse_original = sqrt(mean(result_diff_original.Velocity_D.^2));
    vD_mean_IsZero = mean(result_diff_IsZero.Velocity_D);
    vD_rmse_IsZero = sqrt(mean(result_diff_IsZero.Velocity_D.^2));
    if abs(vD_mean_original) > 1e-10
        vD_mean_reduction = (vD_mean_original - vD_mean_IsZero) / abs(vD_mean_original) * 100;
    else
        vD_mean_reduction = 0;
    end
    if vD_rmse_original > 1e-10
        vD_rmse_reduction = (vD_rmse_original - vD_rmse_IsZero) / vD_rmse_original * 100;
    else
        vD_rmse_reduction = 0;
    end
    fprintf('垂向速度:\n');
    fprintf('  Original:  Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vD_mean_original, vD_rmse_original);
    fprintf('  IsZero:    Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vD_mean_IsZero, vD_rmse_IsZero);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', vD_mean_reduction, vD_rmse_reduction);
    
    % 姿态误差统计
    fprintf('\n【姿态误差】\n');
    
    % 横滚角
    roll_mean_original = mean(result_diff_original.Roll);
    roll_rmse_original = sqrt(mean(result_diff_original.Roll.^2));
    roll_mean_IsZero = mean(result_diff_IsZero.Roll);
    roll_rmse_IsZero = sqrt(mean(result_diff_IsZero.Roll.^2));
    if abs(roll_mean_original) > 1e-10
        roll_mean_reduction = (roll_mean_original - roll_mean_IsZero) / abs(roll_mean_original) * 100;
    else
        roll_mean_reduction = 0;
    end
    if roll_rmse_original > 1e-10
        roll_rmse_reduction = (roll_rmse_original - roll_rmse_IsZero) / roll_rmse_original * 100;
    else
        roll_rmse_reduction = 0;
    end
    fprintf('横滚角:\n');
    fprintf('  Original:  Mean = %12.6f deg,  RMSE = %12.6f deg\n', roll_mean_original, roll_rmse_original);
    fprintf('  IsZero:    Mean = %12.6f deg,  RMSE = %12.6f deg\n', roll_mean_IsZero, roll_rmse_IsZero);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', roll_mean_reduction, roll_rmse_reduction);
    
    % 俯仰角
    pitch_mean_original = mean(result_diff_original.Pitch);
    pitch_rmse_original = sqrt(mean(result_diff_original.Pitch.^2));
    pitch_mean_IsZero = mean(result_diff_IsZero.Pitch);
    pitch_rmse_IsZero = sqrt(mean(result_diff_IsZero.Pitch.^2));
    if abs(pitch_mean_original) > 1e-10
        pitch_mean_reduction = (pitch_mean_original - pitch_mean_IsZero) / abs(pitch_mean_original) * 100;
    else
        pitch_mean_reduction = 0;
    end
    if pitch_rmse_original > 1e-10
        pitch_rmse_reduction = (pitch_rmse_original - pitch_rmse_IsZero) / pitch_rmse_original * 100;
    else
        pitch_rmse_reduction = 0;
    end
    fprintf('俯仰角:\n');
    fprintf('  Original:  Mean = %12.6f deg,  RMSE = %12.6f deg\n', pitch_mean_original, pitch_rmse_original);
    fprintf('  IsZero:    Mean = %12.6f deg,  RMSE = %12.6f deg\n', pitch_mean_IsZero, pitch_rmse_IsZero);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', pitch_mean_reduction, pitch_rmse_reduction);
    
    % 航向角
    yaw_mean_original = mean(result_diff_original.Yaw);
    yaw_rmse_original = sqrt(mean(result_diff_original.Yaw.^2));
    yaw_mean_IsZero = mean(result_diff_IsZero.Yaw);
    yaw_rmse_IsZero = sqrt(mean(result_diff_IsZero.Yaw.^2));
    if abs(yaw_mean_original) > 1e-10
        yaw_mean_reduction = (yaw_mean_original - yaw_mean_IsZero) / abs(yaw_mean_original) * 100;
    else
        yaw_mean_reduction = 0;
    end
    if yaw_rmse_original > 1e-10
        yaw_rmse_reduction = (yaw_rmse_original - yaw_rmse_IsZero) / yaw_rmse_original * 100;
    else
        yaw_rmse_reduction = 0;
    end
    fprintf('航向角:\n');
    fprintf('  Original:  Mean = %12.6f deg,  RMSE = %12.6f deg\n', yaw_mean_original, yaw_rmse_original);
    fprintf('  IsZero:    Mean = %12.6f deg,  RMSE = %12.6f deg\n', yaw_mean_IsZero, yaw_rmse_IsZero);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', yaw_mean_reduction, yaw_rmse_reduction);
    
    fprintf('==================================\n\n');
    
    fprintf('对比图表绘制完成！\n');
end

