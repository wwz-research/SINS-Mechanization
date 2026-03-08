% 绘制陀螺仪原始数据与补偿数据的函数
function Align_fig(data_1, data_2, data_3, mode)
    switch(mode)
        case 0
            % 提取整段数据的对准结果
            whole_yaw = data_1(:, 1);
            whole_pitch = data_1(:, 2);
            whole_roll = data_1(:, 3);
            fprintf("对准平均结果 俯仰(yaw)：%f 航向(pitch)：%f 横滚(roll)：%f\n", whole_yaw, whole_pitch, whole_roll);
        
            % 提取每秒平均的对准结果
            seconds = data_2(:, 1);
            second_yaw = data_2(:, 2);
            second_pitch = data_2(:, 3);
            second_roll = data_2(:, 4);
        
            % 提取每个历元的平均对准结果
            SecOfWeek = table2array(data_3(:, 1));
            First = SecOfWeek(1);
            SecOfWeek = SecOfWeek - First;
            epoch_yaw = table2array(data_3(:, 2));
            epoch_pitch = table2array(data_3(:, 3));
            epoch_roll = table2array(data_3(:, 4));
            
            % 计算最大偏移量
            Max_syaw = max(abs(second_yaw)) - abs(whole_yaw);    % 每秒
            Max_spitch = max(abs(second_pitch)) - abs(whole_pitch);
            Max_sroll = max(abs(second_roll)) - abs(whole_roll);
            Max_eyaw = max(abs(epoch_yaw)) - abs(whole_yaw);     % 每历元
            Max_epitch = max(abs(epoch_pitch)) - abs(whole_pitch);
            Max_eroll = max(abs(epoch_roll)) - abs(whole_roll);
        
            fprintf("每秒最大偏移量 俯仰(yaw)：%f 航向(pitch)：%f 横滚(roll)：%f\n", Max_syaw, Max_spitch, Max_sroll);
            fprintf("每历元最大偏移量 俯仰(yaw)：%f 航向(pitch)：%f 横滚(roll)：%f\n", Max_eyaw, Max_epitch, Max_eroll);
        
            % 绘制    
            figure;
            % Epoch Yaw
            plot(SecOfWeek, epoch_yaw, 'r', 'DisplayName', 'yaw epoch')  % 每历元
            hold on;
            plot([seconds(1), seconds(end)], [whole_yaw, whole_yaw], 'k--', 'DisplayName', 'Avg');
            title('Every Epoch Yaw');
            xlabel('Second(s)');
            ylabel('Yaw (Deg)');
            legend;
            grid on;
        
            % Epoch Pitch
            figure;
            plot(SecOfWeek, epoch_pitch, 'b', 'DisplayName', 'pitch epoch')  % 每历元    
            hold on;
            plot([seconds(1), seconds(end)], [whole_pitch, whole_pitch], 'k--','DisplayName', 'Avg');
            title('Every Epoch Pitch');
            xlabel('Second(s)');
            ylabel('Pitch (Deg)');
            legend;
            grid on;
        
            % Epoch Roll
            figure;
            plot(SecOfWeek, epoch_roll, 'g', 'DisplayName', 'roll epoch')  % 每历元
            hold on;
            plot([seconds(1), seconds(end)], [whole_roll, whole_roll], 'k--');
            title('Every Epoch Roll');
            xlabel('Second(s)');
            ylabel('Roll (Deg)');
            legend;
            grid on;
        
            figure;
            % Second Yaw
            plot(seconds, second_yaw, 'r', 'DisplayName', 'yaw second');   % 每秒
            hold on;
            plot([seconds(1), seconds(end)], [whole_yaw, whole_yaw], 'k--', 'DisplayName', 'Avg');
            title('Every Seconds Yaw');
            xlabel('Second(s)');
            ylabel('Yaw (Deg)');
            legend;
            grid on;
        
            % Second Pitch
            figure;
            plot(seconds, second_pitch, 'b', 'DisplayName', 'pitch second');
            hold on;
            plot([seconds(1), seconds(end)], [whole_pitch, whole_pitch], 'k--','DisplayName', 'Avg');
            title('Every Seconds Pitch');
            xlabel('Second(s)');
            ylabel('Pitch (Deg)');
            legend;
            grid on;
        
            % Second Roll
            figure;
            plot(seconds, second_roll, 'g', 'DisplayName', 'roll second');
            hold on;
            plot([seconds(1), seconds(end)], [whole_roll, whole_roll], 'k--');
            title('Every Seconds Roll');
            xlabel('Second(s)');
            ylabel('Roll (Deg)');
            legend;
            grid on;
    
        case 1
            % 提取白噪声随时间变化
            time =  table2array(data_1(:, 1));
            noise =  table2array(data_1(:, 2));
    
            % 查找接近时间 1 秒的索引
            [~, idx] = min(abs(time - 1));  % 找到与 1 秒最接近的索引
            
            % 输出时间为 1 秒时的白噪声误差
            disp(['白噪声误差在 1 秒时的值: ', num2str(noise(idx))]);
            % 绘制    
            figure;
            plot(time, noise, 'r', 'DisplayName', 'White Noise')
            hold on;
            % 绘制纵坐标值为 0.221123558303 的直线
            yline(0.221123558303, 'k--', 'DisplayName', 'Bias Noise');  % 使用黑色虚线绘制，并标注
    
            title('White Noise change with time');
            xlabel('Second(s)');
            ylabel('Deg');
            legend;
            grid on;
    end
end