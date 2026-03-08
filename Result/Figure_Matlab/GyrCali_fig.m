% 绘制陀螺仪原始数据与补偿数据的函数
function GyrCali_fig(data, mode)
    % 时间列
    SecOfWeek = data(:, 1);

    % 补偿后的角速度数据
    com = data(:, 2);

    % 原始的角速度的数据
    raw = data(:, 3);

    % 计算平均值并打印
    avg_com = mean(com);
    avg_raw = mean(raw);
    fprintf('补偿前：%.10f rad/s 补偿后：%.10f rad/s\n', avg_raw, avg_com);

    % 角速度10deg/s转化为弧度每秒
    deg_per_sec = 10; % 角速度10deg/s
    rad_per_sec = deg_per_sec * pi / 180; % 转换为弧度每秒
    fprintf('角速度 10 deg/s 转化为 %.10f rad/s\n', rad_per_sec);

    % 创建一个新的图形窗口
    figure;

    % 绘制补偿后的角速度数据
    plot(SecOfWeek, com, 'r', 'DisplayName', 'Compensated');
    hold on;
    plot(SecOfWeek, raw, 'b', 'DisplayName', 'Raw');

    % 根据mode绘制角速度线
    if mode == 0
        plot([SecOfWeek(1), SecOfWeek(end)], [-rad_per_sec, -rad_per_sec], 'k--', 'LineWidth', 1, 'DisplayName', '-10 deg/s');
        % 标注负角速度
        text(SecOfWeek(end)*0.8, -rad_per_sec, '-10 deg/s = -0.1745 rad/s', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
    elseif mode == 1
        plot([SecOfWeek(1), SecOfWeek(end)], [rad_per_sec, rad_per_sec], 'k--', 'LineWidth', 1, 'DisplayName', '+10 deg/s');
        % 标注正角速度
        text(SecOfWeek(end)*0.8, rad_per_sec, '+10 deg/s = 0.1745 rad/s', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
    end

    % 显示平均值
%     plot([SecOfWeek(1), SecOfWeek(end)], [avg_com, avg_com], 'k--', 'DisplayName', 'Avg Compensated');
%     plot([SecOfWeek(1), SecOfWeek(end)], [avg_raw, avg_raw], 'g--', 'DisplayName', 'Avg Raw');

    % 设置图表标题和标签
    title('Gyroscope Data Compensation vs Raw');
    xlabel('SecOfWeek (s)');
    ylabel('Angular Velocity (rad/s)');
    legend('show');
    grid on;
end
