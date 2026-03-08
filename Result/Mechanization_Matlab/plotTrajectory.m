function plotTrajectory(result_denu)
% 绘制轨迹对比图
% 输入参数：
%   result_denu - 轨迹数据（table格式，包含E_Result、N_Result、U_Result、E_Ref、N_Ref、U_Ref列）
% 输出：
%   figure2: 平面轨迹对比图（E-N平面）
%   figure3: 垂向轨迹对比图（U方向时序图）

    % 提取时间序列
    time = result_denu.GPSTime;

    % ========== Figure 2: 平面轨迹对比图（E-N平面） ==========
    figure(2);
    clf;
    
    % 绘制平面轨迹
    plot(result_denu.E_Result, result_denu.N_Result, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算轨迹');
    hold on;
    plot(result_denu.E_Ref, result_denu.N_Ref, 'r--', 'LineWidth', 1.5, 'DisplayName', '真值轨迹');
    xlabel('E (m)');
    ylabel('N (m)');
    title('平面轨迹对比（E-N平面）');
    legend('Location', 'best');
    grid on;
    axis equal;
    
    % ========== Figure 3: 垂向轨迹对比图（U方向时序） ==========
    figure(3);
    clf;
    
    % 绘制U方向时序对比
    plot(time, result_denu.U_Result, 'b-', 'LineWidth', 1.5, 'DisplayName', '解算轨迹');
    hold on;
    plot(time, result_denu.U_Ref, 'r--', 'LineWidth', 1.5, 'DisplayName', '真值轨迹');
    xlabel('GPS周秒');
    ylabel('U (m)');
    title('垂向轨迹对比');
    legend('Location', 'best');
    grid on;
end
