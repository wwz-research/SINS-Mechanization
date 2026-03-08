function compareVchange()
% 对比Mechanization_vchange和Mechanization_IsZero的误差时序图并计算误差下降百分比
% 功能：
%   1. 读取Mechanization_vchange和Mechanization_IsZero的误差数据
%   2. 在同一子图中绘制Mechanization_vchange和Mechanization_IsZero的对比曲线
%   3. 计算Mechanization_vchange误差的Mean、RMSE相较于Mechanization_IsZero误差下降多少

    clear;
    close all;
    clc;

    % 设置数据文件路径
    dataPath_IsZero = 'Mechanization_IsZero';
    dataPath_vchange = 'Mechanization_vchange';
    
    resultDiffFile_IsZero = fullfile(dataPath_IsZero, 'result_diff.txt');
    resultDiffFile_vchange = fullfile(dataPath_vchange, 'result_diff.txt');

    % 读取数据
    fprintf('正在读取数据文件...\n');
    result_diff_IsZero = readDataDiff(resultDiffFile_IsZero);
    result_diff_vchange = readDataDiff(resultDiffFile_vchange);

    % 检查数据长度是否一致，使用最短长度
    lengths = [height(result_diff_IsZero), height(result_diff_vchange)];
    minLen = min(lengths);
    if length(unique(lengths)) > 1
        warning('数据长度不一致，将使用最短长度 %d', minLen);
    end
    
    result_diff_IsZero = result_diff_IsZero(1:minLen, :);
    result_diff_vchange = result_diff_vchange(1:minLen, :);

    % 提取时间序列（GPS周秒）
    time = result_diff_IsZero.GPSTime;

    fprintf('数据读取完成，共 %d 个数据点\n', length(time));

    % 处理航向角误差：解决取值范围突变问题
    % 大于300的减去360，小于-300的加上360
    result_diff_IsZero = processYawError(result_diff_IsZero);
    result_diff_vchange = processYawError(result_diff_vchange);

    % 创建对比图
    figure(7);
    clf;
    
    % ========== 第一行：ENU坐标系误差 ==========
    % dE误差
    subplot(3,3,1);
    plot(time, result_diff_IsZero.dE, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_vchange.dE, 'r--', 'LineWidth', 2, 'Marker', 's', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'vchange');
    xlabel('GPS周秒');
    ylabel('dE (m)');
    title('E方向误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % dN误差
    subplot(3,3,2);
    plot(time, result_diff_IsZero.dN, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_vchange.dN, 'r--', 'LineWidth', 2, 'Marker', 's', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'vchange');
    xlabel('GPS周秒');
    ylabel('dN (m)');
    title('N方向误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % dU误差
    subplot(3,3,3);
    plot(time, result_diff_IsZero.dU, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_vchange.dU, 'r--', 'LineWidth', 2, 'Marker', 's', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'vchange');
    xlabel('GPS周秒');
    ylabel('dU (m)');
    title('U方向误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % ========== 第二行：速度误差 ==========
    % 北向速度误差
    subplot(3,3,4);
    plot(time, result_diff_IsZero.Velocity_N, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_vchange.Velocity_N, 'r--', 'LineWidth', 2, 'Marker', 's', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'vchange');
    xlabel('GPS周秒');
    ylabel('北向速度误差 (m/s)');
    title('北向速度误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 东向速度误差
    subplot(3,3,5);
    plot(time, result_diff_IsZero.Velocity_E, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_vchange.Velocity_E, 'r--', 'LineWidth', 2, 'Marker', 's', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'vchange');
    xlabel('GPS周秒');
    ylabel('东向速度误差 (m/s)');
    title('东向速度误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 垂向速度误差
    subplot(3,3,6);
    plot(time, result_diff_IsZero.Velocity_D, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_vchange.Velocity_D, 'r--', 'LineWidth', 2, 'Marker', 's', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'vchange');
    xlabel('GPS周秒');
    ylabel('垂向速度误差 (m/s)');
    title('垂向速度误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % ========== 第三行：姿态误差 ==========
    % 横滚角误差
    subplot(3,3,7);
    plot(time, result_diff_IsZero.Roll, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_vchange.Roll, 'r--', 'LineWidth', 2, 'Marker', 's', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'vchange');
    xlabel('GPS周秒');
    ylabel('横滚角误差 (deg)');
    title('横滚角误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 俯仰角误差
    subplot(3,3,8);
    plot(time, result_diff_IsZero.Pitch, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_vchange.Pitch, 'r--', 'LineWidth', 2, 'Marker', 's', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'vchange');
    xlabel('GPS周秒');
    ylabel('俯仰角误差 (deg)');
    title('俯仰角误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 航向角误差
    subplot(3,3,9);
    plot(time, result_diff_IsZero.Yaw, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'IsZero');
    hold on;
    plot(time, result_diff_vchange.Yaw, 'r--', 'LineWidth', 2, 'Marker', 's', 'MarkerIndices', 1:50:length(time), ...
         'MarkerSize', 4, 'DisplayName', 'vchange');
    xlabel('GPS周秒');
    ylabel('航向角误差 (deg)');
    title('航向角误差');
    legend('Location', 'best');
    grid on;
    hold off;
    
    sgtitle('误差时序对比图 (IsZero vs vchange)', 'FontSize', 14, 'FontWeight', 'bold');

    % ========== 计算并输出统计指标和下降百分比 ==========
    fprintf('\n========== 误差统计指标对比（vchange相对于IsZero） ==========\n');
    
    % ENU坐标系误差统计
    fprintf('\n【ENU坐标系误差】\n');
    
    % dE
    dE_mean_IsZero = mean(result_diff_IsZero.dE);
    dE_rmse_IsZero = sqrt(mean(result_diff_IsZero.dE.^2));
    dE_mean_vchange = mean(result_diff_vchange.dE);
    dE_rmse_vchange = sqrt(mean(result_diff_vchange.dE.^2));
    if abs(dE_mean_IsZero) > 1e-10
        dE_mean_reduction = (dE_mean_IsZero - dE_mean_vchange) / abs(dE_mean_IsZero) * 100;
    else
        dE_mean_reduction = 0;
    end
    if dE_rmse_IsZero > 1e-10
        dE_rmse_reduction = (dE_rmse_IsZero - dE_rmse_vchange) / dE_rmse_IsZero * 100;
    else
        dE_rmse_reduction = 0;
    end
    fprintf('dE:\n');
    fprintf('  IsZero:  Mean = %12.6f m,  RMSE = %12.6f m\n', dE_mean_IsZero, dE_rmse_IsZero);
    fprintf('  vchange:   Mean = %12.6f m,  RMSE = %12.6f m\n', dE_mean_vchange, dE_rmse_vchange);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', dE_mean_reduction, dE_rmse_reduction);
    
    % dN
    dN_mean_IsZero = mean(result_diff_IsZero.dN);
    dN_rmse_IsZero = sqrt(mean(result_diff_IsZero.dN.^2));
    dN_mean_vchange = mean(result_diff_vchange.dN);
    dN_rmse_vchange = sqrt(mean(result_diff_vchange.dN.^2));
    if abs(dN_mean_IsZero) > 1e-10
        dN_mean_reduction = (dN_mean_IsZero - dN_mean_vchange) / abs(dN_mean_IsZero) * 100;
    else
        dN_mean_reduction = 0;
    end
    if dN_rmse_IsZero > 1e-10
        dN_rmse_reduction = (dN_rmse_IsZero - dN_rmse_vchange) / dN_rmse_IsZero * 100;
    else
        dN_rmse_reduction = 0;
    end
    fprintf('dN:\n');
    fprintf('  IsZero:  Mean = %12.6f m,  RMSE = %12.6f m\n', dN_mean_IsZero, dN_rmse_IsZero);
    fprintf('  vchange:   Mean = %12.6f m,  RMSE = %12.6f m\n', dN_mean_vchange, dN_rmse_vchange);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', dN_mean_reduction, dN_rmse_reduction);
    
    % dU
    dU_mean_IsZero = mean(result_diff_IsZero.dU);
    dU_rmse_IsZero = sqrt(mean(result_diff_IsZero.dU.^2));
    dU_mean_vchange = mean(result_diff_vchange.dU);
    dU_rmse_vchange = sqrt(mean(result_diff_vchange.dU.^2));
    if abs(dU_mean_IsZero) > 1e-10
        dU_mean_reduction = (dU_mean_IsZero - dU_mean_vchange) / abs(dU_mean_IsZero) * 100;
    else
        dU_mean_reduction = 0;
    end
    if dU_rmse_IsZero > 1e-10
        dU_rmse_reduction = (dU_rmse_IsZero - dU_rmse_vchange) / dU_rmse_IsZero * 100;
    else
        dU_rmse_reduction = 0;
    end
    fprintf('dU:\n');
    fprintf('  IsZero:  Mean = %12.6f m,  RMSE = %12.6f m\n', dU_mean_IsZero, dU_rmse_IsZero);
    fprintf('  vchange:   Mean = %12.6f m,  RMSE = %12.6f m\n', dU_mean_vchange, dU_rmse_vchange);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', dU_mean_reduction, dU_rmse_reduction);
    
    % 速度误差统计
    fprintf('\n【速度误差】\n');
    
    % 北向速度
    vN_mean_IsZero = mean(result_diff_IsZero.Velocity_N);
    vN_rmse_IsZero = sqrt(mean(result_diff_IsZero.Velocity_N.^2));
    vN_mean_vchange = mean(result_diff_vchange.Velocity_N);
    vN_rmse_vchange = sqrt(mean(result_diff_vchange.Velocity_N.^2));
    if abs(vN_mean_IsZero) > 1e-10
        vN_mean_reduction = (vN_mean_IsZero - vN_mean_vchange) / abs(vN_mean_IsZero) * 100;
    else
        vN_mean_reduction = 0;
    end
    if vN_rmse_IsZero > 1e-10
        vN_rmse_reduction = (vN_rmse_IsZero - vN_rmse_vchange) / vN_rmse_IsZero * 100;
    else
        vN_rmse_reduction = 0;
    end
    fprintf('北向速度:\n');
    fprintf('  IsZero:  Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vN_mean_IsZero, vN_rmse_IsZero);
    fprintf('  vchange:   Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vN_mean_vchange, vN_rmse_vchange);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', vN_mean_reduction, vN_rmse_reduction);
    
    % 东向速度
    vE_mean_IsZero = mean(result_diff_IsZero.Velocity_E);
    vE_rmse_IsZero = sqrt(mean(result_diff_IsZero.Velocity_E.^2));
    vE_mean_vchange = mean(result_diff_vchange.Velocity_E);
    vE_rmse_vchange = sqrt(mean(result_diff_vchange.Velocity_E.^2));
    if abs(vE_mean_IsZero) > 1e-10
        vE_mean_reduction = (vE_mean_IsZero - vE_mean_vchange) / abs(vE_mean_IsZero) * 100;
    else
        vE_mean_reduction = 0;
    end
    if vE_rmse_IsZero > 1e-10
        vE_rmse_reduction = (vE_rmse_IsZero - vE_rmse_vchange) / vE_rmse_IsZero * 100;
    else
        vE_rmse_reduction = 0;
    end
    fprintf('东向速度:\n');
    fprintf('  IsZero:  Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vE_mean_IsZero, vE_rmse_IsZero);
    fprintf('  vchange:   Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vE_mean_vchange, vE_rmse_vchange);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', vE_mean_reduction, vE_rmse_reduction);
    
    % 垂向速度
    vD_mean_IsZero = mean(result_diff_IsZero.Velocity_D);
    vD_rmse_IsZero = sqrt(mean(result_diff_IsZero.Velocity_D.^2));
    vD_mean_vchange = mean(result_diff_vchange.Velocity_D);
    vD_rmse_vchange = sqrt(mean(result_diff_vchange.Velocity_D.^2));
    if abs(vD_mean_IsZero) > 1e-10
        vD_mean_reduction = (vD_mean_IsZero - vD_mean_vchange) / abs(vD_mean_IsZero) * 100;
    else
        vD_mean_reduction = 0;
    end
    if vD_rmse_IsZero > 1e-10
        vD_rmse_reduction = (vD_rmse_IsZero - vD_rmse_vchange) / vD_rmse_IsZero * 100;
    else
        vD_rmse_reduction = 0;
    end
    fprintf('垂向速度:\n');
    fprintf('  IsZero:  Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vD_mean_IsZero, vD_rmse_IsZero);
    fprintf('  vchange:   Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vD_mean_vchange, vD_rmse_vchange);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', vD_mean_reduction, vD_rmse_reduction);
    
    % 姿态误差统计
    fprintf('\n【姿态误差】\n');
    
    % 横滚角
    roll_mean_IsZero = mean(result_diff_IsZero.Roll);
    roll_rmse_IsZero = sqrt(mean(result_diff_IsZero.Roll.^2));
    roll_mean_vchange = mean(result_diff_vchange.Roll);
    roll_rmse_vchange = sqrt(mean(result_diff_vchange.Roll.^2));
    if abs(roll_mean_IsZero) > 1e-10
        roll_mean_reduction = (roll_mean_IsZero - roll_mean_vchange) / abs(roll_mean_IsZero) * 100;
    else
        roll_mean_reduction = 0;
    end
    if roll_rmse_IsZero > 1e-10
        roll_rmse_reduction = (roll_rmse_IsZero - roll_rmse_vchange) / roll_rmse_IsZero * 100;
    else
        roll_rmse_reduction = 0;
    end
    fprintf('横滚角:\n');
    fprintf('  IsZero:  Mean = %12.6f deg,  RMSE = %12.6f deg\n', roll_mean_IsZero, roll_rmse_IsZero);
    fprintf('  vchange:   Mean = %12.6f deg,  RMSE = %12.6f deg\n', roll_mean_vchange, roll_rmse_vchange);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', roll_mean_reduction, roll_rmse_reduction);
    
    % 俯仰角
    pitch_mean_IsZero = mean(result_diff_IsZero.Pitch);
    pitch_rmse_IsZero = sqrt(mean(result_diff_IsZero.Pitch.^2));
    pitch_mean_vchange = mean(result_diff_vchange.Pitch);
    pitch_rmse_vchange = sqrt(mean(result_diff_vchange.Pitch.^2));
    if abs(pitch_mean_IsZero) > 1e-10
        pitch_mean_reduction = (pitch_mean_IsZero - pitch_mean_vchange) / abs(pitch_mean_IsZero) * 100;
    else
        pitch_mean_reduction = 0;
    end
    if pitch_rmse_IsZero > 1e-10
        pitch_rmse_reduction = (pitch_rmse_IsZero - pitch_rmse_vchange) / pitch_rmse_IsZero * 100;
    else
        pitch_rmse_reduction = 0;
    end
    fprintf('俯仰角:\n');
    fprintf('  IsZero:  Mean = %12.6f deg,  RMSE = %12.6f deg\n', pitch_mean_IsZero, pitch_rmse_IsZero);
    fprintf('  vchange:   Mean = %12.6f deg,  RMSE = %12.6f deg\n', pitch_mean_vchange, pitch_rmse_vchange);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', pitch_mean_reduction, pitch_rmse_reduction);
    
    % 航向角
    yaw_mean_IsZero = mean(result_diff_IsZero.Yaw);
    yaw_rmse_IsZero = sqrt(mean(result_diff_IsZero.Yaw.^2));
    yaw_mean_vchange = mean(result_diff_vchange.Yaw);
    yaw_rmse_vchange = sqrt(mean(result_diff_vchange.Yaw.^2));
    if abs(yaw_mean_IsZero) > 1e-10
        yaw_mean_reduction = (yaw_mean_IsZero - yaw_mean_vchange) / abs(yaw_mean_IsZero) * 100;
    else
        yaw_mean_reduction = 0;
    end
    if yaw_rmse_IsZero > 1e-10
        yaw_rmse_reduction = (yaw_rmse_IsZero - yaw_rmse_vchange) / yaw_rmse_IsZero * 100;
    else
        yaw_rmse_reduction = 0;
    end
    fprintf('航向角:\n');
    fprintf('  IsZero:  Mean = %12.6f deg,  RMSE = %12.6f deg\n', yaw_mean_IsZero, yaw_rmse_IsZero);
    fprintf('  vchange:   Mean = %12.6f deg,  RMSE = %12.6f deg\n', yaw_mean_vchange, yaw_rmse_vchange);
    fprintf('  下降百分比:  Mean = %6.2f%%,  RMSE = %6.2f%%\n', yaw_mean_reduction, yaw_rmse_reduction);
    
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

