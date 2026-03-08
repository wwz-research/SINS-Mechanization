function plotAllError(result_diff, time)
% 绘制所有误差时序图并计算统计指标
% 输入参数：
%   result_diff - 差分结果数据（table格式，需包含dE、dN、dU列）
%   time - 时间序列（GPS周秒）
% 输出：
%   figure4: ENU坐标系、速度、姿态误差时序图（3行3列）
%   并输出所有误差的means和RMSE

    % 检查是否包含dE、dN、dU列
    if ~ismember('dE', result_diff.Properties.VariableNames) || ...
       ~ismember('dN', result_diff.Properties.VariableNames) || ...
       ~ismember('dU', result_diff.Properties.VariableNames)
        error('result_diff数据中不包含dE、dN、dU列，请使用readDataDiff函数读取数据');
    end

    % 处理航向角误差：解决取值范围突变问题
    % 大于300的减去360，小于-300的加上360
    if ismember('Yaw', result_diff.Properties.VariableNames)
        yaw_error = result_diff.Yaw;
        yaw_error(yaw_error > 300) = yaw_error(yaw_error > 300) - 360;
        yaw_error(yaw_error < -300) = yaw_error(yaw_error < -300) + 360;
        result_diff.Yaw = yaw_error;
    end

    figure(4);
    clf;
    
    % ========== 第一行：ENU坐标系误差 ==========
    % dE误差
    subplot(3,3,1);
    plot(time, result_diff.dE, 'r-', 'LineWidth', 1.5);
    xlabel('GPS周秒');
    ylabel('dE (m)');
    title('E方向误差');
    grid on;
    
    % dN误差
    subplot(3,3,2);
    plot(time, result_diff.dN, 'r-', 'LineWidth', 1.5);
    xlabel('GPS周秒');
    ylabel('dN (m)');
    title('N方向误差');
    grid on;
    
    % dU误差
    subplot(3,3,3);
    plot(time, result_diff.dU, 'r-', 'LineWidth', 1.5);
    xlabel('GPS周秒');
    ylabel('dU (m)');
    title('U方向误差');
    grid on;
    
    % ========== 第二行：速度误差 ==========
    % 北向速度误差
    subplot(3,3,4);
    plot(time, result_diff.Velocity_N, 'b-', 'LineWidth', 1.5);
    xlabel('GPS周秒');
    ylabel('北向速度误差 (m/s)');
    title('北向速度误差');
    grid on;
    
    % 东向速度误差
    subplot(3,3,5);
    plot(time, result_diff.Velocity_E, 'b-', 'LineWidth', 1.5);
    xlabel('GPS周秒');
    ylabel('东向速度误差 (m/s)');
    title('东向速度误差');
    grid on;
    
    % 垂向速度误差
    subplot(3,3,6);
    plot(time, result_diff.Velocity_D, 'b-', 'LineWidth', 1.5);
    xlabel('GPS周秒');
    ylabel('垂向速度误差 (m/s)');
    title('垂向速度误差');
    grid on;
    
    % ========== 第三行：姿态误差 ==========
    % 横滚角误差
    subplot(3,3,7);
    plot(time, result_diff.Roll, 'g-', 'LineWidth', 1.5);
    xlabel('GPS周秒');
    ylabel('横滚角误差 (deg)');
    title('横滚角误差');
    grid on;
    
    % 俯仰角误差
    subplot(3,3,8);
    plot(time, result_diff.Pitch, 'g-', 'LineWidth', 1.5);
    xlabel('GPS周秒');
    ylabel('俯仰角误差 (deg)');
    title('俯仰角误差');
    grid on;
    
    % 航向角误差
    subplot(3,3,9);
    plot(time, result_diff.Yaw, 'g-', 'LineWidth', 1.5);
    xlabel('GPS周秒');
    ylabel('航向角误差 (deg)');
    title('航向角误差');
    grid on;
    
    sgtitle('误差时序图', 'FontSize', 14, 'FontWeight', 'bold');
    
    % ========== 计算并输出统计指标 ==========
    fprintf('\n========== 误差统计指标 ==========\n');
    
    % ENU坐标系误差统计
    fprintf('\n【ENU坐标系误差】\n');
    dE_mean = mean(result_diff.dE);
    dE_rmse = sqrt(mean(result_diff.dE.^2));
    fprintf('dE:  Mean = %12.6f m,  RMSE = %12.6f m\n', dE_mean, dE_rmse);
    
    dN_mean = mean(result_diff.dN);
    dN_rmse = sqrt(mean(result_diff.dN.^2));
    fprintf('dN:  Mean = %12.6f m,  RMSE = %12.6f m\n', dN_mean, dN_rmse);
    
    dU_mean = mean(result_diff.dU);
    dU_rmse = sqrt(mean(result_diff.dU.^2));
    fprintf('dU:  Mean = %12.6f m,  RMSE = %12.6f m\n', dU_mean, dU_rmse);
    
    % 速度误差统计
    fprintf('\n【速度误差】\n');
    vN_mean = mean(result_diff.Velocity_N);
    vN_rmse = sqrt(mean(result_diff.Velocity_N.^2));
    fprintf('北向速度:  Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vN_mean, vN_rmse);
    
    vE_mean = mean(result_diff.Velocity_E);
    vE_rmse = sqrt(mean(result_diff.Velocity_E.^2));
    fprintf('东向速度:  Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vE_mean, vE_rmse);
    
    vD_mean = mean(result_diff.Velocity_D);
    vD_rmse = sqrt(mean(result_diff.Velocity_D.^2));
    fprintf('垂向速度:  Mean = %12.6f m/s,  RMSE = %12.6f m/s\n', vD_mean, vD_rmse);
    
    % 姿态误差统计
    fprintf('\n【姿态误差】\n');
    roll_mean = mean(result_diff.Roll);
    roll_rmse = sqrt(mean(result_diff.Roll.^2));
    fprintf('横滚角:  Mean = %12.6f deg,  RMSE = %12.6f deg\n', roll_mean, roll_rmse);
    
    pitch_mean = mean(result_diff.Pitch);
    pitch_rmse = sqrt(mean(result_diff.Pitch.^2));
    fprintf('俯仰角:  Mean = %12.6f deg,  RMSE = %12.6f deg\n', pitch_mean, pitch_rmse);
    
    yaw_mean = mean(result_diff.Yaw);
    yaw_rmse = sqrt(mean(result_diff.Yaw.^2));
    fprintf('航向角:  Mean = %12.6f deg,  RMSE = %12.6f deg\n', yaw_mean, yaw_rmse);
    
    fprintf('==================================\n\n');
end

