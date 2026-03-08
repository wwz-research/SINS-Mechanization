% 绘制加速度计原始数据与补偿数据的函数
function AccCali_fig(data, axis, mode)
    % 时间列
    SecOfWeek = data(:, 1);
    
    % 补偿后加速度数据
    c_x = data(:, 2);
    c_y = data(:, 3);
    c_z = data(:, 4);
    
    % 原始加速度数据
    r_x = data(:, 5);
    r_y = data(:, 6);
    r_z = data(:, 7);
    
    % 计算平均值并打印
    avg_c_x = mean(c_x);
    avg_c_y = mean(c_y);
    avg_c_z = mean(c_z);
    avg_r_x = mean(r_x);
    avg_r_y = mean(r_y);
    avg_r_z = mean(r_z);
    fprintf('补偿前：%.10f m/s² %.10f m/s² %.10f m/s² ', avg_r_x, avg_r_y, avg_r_z);
    fprintf('补偿后：%.10f m/s² %.10f m/s² %.10f m/s²\n', avg_c_x, avg_c_y, avg_c_z);

    % 创建一个新的图形窗口
    figure;

    % 绘制 X 轴的数据
    subplot(3, 1, 1); % 上图：X轴数据
    plot(SecOfWeek, c_x, 'r', 'DisplayName', 'Compensated');
    hold on;
    plot(SecOfWeek, r_x, 'b', 'DisplayName', 'Raw');
    
    % 绘制铅锤重力线
    if axis == 0
        if mode == 0
            plot([SecOfWeek(1), SecOfWeek(end)], [9.7936174, 9.7936174], 'g-', 'LineWidth', 2, 'DisplayName', 'g = 9.7936174');
            % 添加注释
            text(SecOfWeek(end)*0.8, 9.7936174, 'g = 9.7936174', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
        elseif mode == 1
            plot([SecOfWeek(1), SecOfWeek(end)], [-9.7936174, -9.7936174], 'g-', 'LineWidth', 2, 'DisplayName', '-g = -9.7936174');
            % 添加注释
            text(SecOfWeek(end)*0.8, -9.7936174, '-g = -9.7936174', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
        end
    end
    
    % 显示补偿后的平均值
    plot([SecOfWeek(1), SecOfWeek(end)], [avg_c_x, avg_c_x], 'k--', 'DisplayName', 'Avg Compensated');
    plot([SecOfWeek(1), SecOfWeek(end)], [avg_r_x, avg_r_x], 'g--', 'DisplayName', 'Avg Raw');
    
    title('Accelerometer Data Compensation vs Raw');
    xlabel('SecOfWeek (s)');
    ylabel('X Acceleration (m/s^2)');
    legend;
    set(gca, 'XTick', [], 'XLabel', []);  % 去掉x轴刻度和标签
    
    % 绘制 Y 轴的数据
    subplot(3, 1, 2); % 中图：Y轴数据
    plot(SecOfWeek, c_y, 'r');
    hold on;
    plot(SecOfWeek, r_y, 'b');
    
    % 绘制铅锤重力线
    if axis == 1
        if mode == 0
            plot([SecOfWeek(1), SecOfWeek(end)], [9.7936174, 9.7936174], 'g-', 'LineWidth', 2, 'DisplayName', 'g = 9.7936174');
            % 添加注释
            text(SecOfWeek(end)*0.8, 9.7936174, 'g = 9.7936174', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
        elseif mode == 1
            plot([SecOfWeek(1), SecOfWeek(end)], [-9.7936174, -9.7936174], 'g-', 'LineWidth', 2, 'DisplayName', '-g = -9.7936174');
            % 添加注释
            text(SecOfWeek(end)*0.8, -9.7936174, '-g = -9.7936174', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
        end
    end
    
    % 显示补偿后的平均值
    plot([SecOfWeek(1), SecOfWeek(end)], [avg_c_y, avg_c_y], 'k--');
    plot([SecOfWeek(1), SecOfWeek(end)], [avg_r_y, avg_r_y], 'g--');
    
    xlabel('SecOfWeek (s)');
    ylabel('Y Acceleration (m/s^2)');
    set(gca, 'XTick', [], 'XLabel', []);  % 去掉x轴刻度和标签
    
    % 绘制 Z 轴的数据
    subplot(3, 1, 3); % 下图：Z轴数据
    plot(SecOfWeek, c_z, 'r');
    hold on;
    plot(SecOfWeek, r_z, 'b');
    
    % 绘制铅锤重力线
    if axis == 2
        if mode == 0
            plot([SecOfWeek(1), SecOfWeek(end)], [9.7936174, 9.7936174], 'g-', 'LineWidth', 2, 'DisplayName', 'g = 9.7936174');
            % 添加注释
            text(SecOfWeek(end)*0.8, 9.7936174, 'g = 9.7936174', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
        elseif mode == 1
            plot([SecOfWeek(1), SecOfWeek(end)], [-9.7936174, -9.7936174], 'g-', 'LineWidth', 2, 'DisplayName', '-g = -9.7936174');
            % 添加注释
            text(SecOfWeek(end)*0.8, -9.7936174, '-g = -9.7936174', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
        end
    end
    
    % 显示补偿后的平均值
    plot([SecOfWeek(1), SecOfWeek(end)], [avg_c_z, avg_c_z], 'k--');
    plot([SecOfWeek(1), SecOfWeek(end)], [avg_r_z, avg_r_z], 'g--');
    
    xlabel('SecOfWeek (s)');
    ylabel('Z Acceleration (m/s^2)');
    
end
