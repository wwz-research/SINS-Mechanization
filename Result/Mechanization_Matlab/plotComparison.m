function plotComparison(result, result_ref, time)
% 绘制解算结果与参考真值的对比图
% 输入参数：
%   result - 解算结果数据（table格式）
%   result_ref - 参考真值数据（table格式）
%   time - 时间序列（GPS周秒）
% 输出：
%   figure1: 位置、速度、姿态对比图（3行3列）

    figure(1);
    clf;
    
    % ========== 第一行：位置对比 ==========
    % 纬度对比
    subplot(3,3,1);
    plot(time, result.Latitude, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算结果');
    hold on;
    plot(time, result_ref.Latitude, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考真值');
    xlabel('GPS周秒');
    ylabel('纬度 (deg)');
    title('纬度');
    legend('Location', 'best');
    grid on;
    
    % 经度对比
    subplot(3,3,2);
    plot(time, result.Longitude, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算结果');
    hold on;
    plot(time, result_ref.Longitude, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考真值');
    xlabel('GPS周秒');
    ylabel('经度 (deg)');
    title('经度');
    legend('Location', 'best');
    grid on;
    
    % 高度对比
    subplot(3,3,3);
    plot(time, result.Height, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算结果');
    hold on;
    plot(time, result_ref.Height, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考真值');
    xlabel('GPS周秒');
    ylabel('高度 (m)');
    title('高度');
    legend('Location', 'best');
    grid on;
    
    % ========== 第二行：速度对比 ==========
    % 北向速度对比
    subplot(3,3,4);
    plot(time, result.Velocity_N, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算结果');
    hold on;
    plot(time, result_ref.Velocity_N, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考真值');
    xlabel('GPS周秒');
    ylabel('北向速度 (m/s)');
    title('北向速度');
    legend('Location', 'best');
    grid on;
    
    % 东向速度对比
    subplot(3,3,5);
    plot(time, result.Velocity_E, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算结果');
    hold on;
    plot(time, result_ref.Velocity_E, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考真值');
    xlabel('GPS周秒');
    ylabel('东向速度 (m/s)');
    title('东向速度');
    legend('Location', 'best');
    grid on;
    
    % 垂向速度对比
    subplot(3,3,6);
    plot(time, result.Velocity_D, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算结果');
    hold on;
    plot(time, result_ref.Velocity_D, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考真值');
    xlabel('GPS周秒');
    ylabel('垂向速度 (m/s)');
    title('垂向速度');
    legend('Location', 'best');
    grid on;
    
    % ========== 第三行：姿态对比 ==========
    % 横滚角对比
    subplot(3,3,7);
    plot(time, result.Roll, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算结果');
    hold on;
    plot(time, result_ref.Roll, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考真值');
    xlabel('GPS周秒');
    ylabel('横滚角 (deg)');
    title('横滚角');
    legend('Location', 'best');
    grid on;
    
    % 俯仰角对比
    subplot(3,3,8);
    plot(time, result.Pitch, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算结果');
    hold on;
    plot(time, result_ref.Pitch, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考真值');
    xlabel('GPS周秒');
    ylabel('俯仰角 (deg)');
    title('俯仰角');
    legend('Location', 'best');
    grid on;
    
    % 航向角对比
    subplot(3,3,9);
    plot(time, result.Yaw, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算结果');
    hold on;
    plot(time, result_ref.Yaw, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考真值');
    xlabel('GPS周秒');
    ylabel('航向角 (deg)');
    title('航向角');
    legend('Location', 'best');
    grid on;
    
    sgtitle('解算结果与参考真值对比', 'FontSize', 14, 'FontWeight', 'bold');
end
