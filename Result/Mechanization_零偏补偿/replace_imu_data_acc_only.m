%% 参数设置 - 只替换加速度数据
acc_sf = 1.5258789063e-6;   % 加速度计数据转换比例因子
fs     = 100;               % 采样率

acc_file = 'group5_AccCali.txt';
asc_in   = 'group5.ASC';
asc_out  = 'group5_new_acc.ASC';

%% 读取补偿后的加速度
% group5_AccCali.txt: [time, Ax, Ay, Az]
accData = readmatrix(acc_file);   % 默认按逗号分隔
Ax = accData(:,2);
Ay = accData(:,3);
Az = accData(:,4);

%% 物理量 -> 计数（注意是"除以"比例因子和采样率）
accX_cnt = round( Ax ./ (acc_sf * fs) );
accY_cnt = round( Ay ./ (acc_sf * fs) );
accZ_cnt = round( Az ./ (acc_sf * fs) );

%% 坐标轴顺序与 Y 轴符号
% 原始格式：00000077 后依次为
%   Z轴加速度、-Y轴加速度、X轴加速度、Z轴角速度、-Y轴角速度、X轴角速度
newZacc   = accZ_cnt;        % Z
newMinusYacc = -accY_cnt;    % -Y
newXacc   = accX_cnt;        % X

%% 按行读写 ASC 文件，只替换 00000077 后的前3个数值（加速度）
fid_in  = fopen(asc_in, 'r');
if fid_in < 0
    error('无法打开输入文件 %s', asc_in);
end

fid_out = fopen(asc_out, 'w');
if fid_out < 0
    fclose(fid_in);
    error('无法创建输出文件 %s', asc_out);
end

idx = 1;  % 对应补偿数据的行索引

while true
    tline = fgetl(fid_in);
    if ~ischar(tline)
        break;  % 文件结束
    end

    % 用逗号分割整行
    parts = split(string(tline), ',');

    % 查找 '00000077' 所在位置
    idx77 = find(parts == "00000077", 1, 'last');

    if ~isempty(idx77)
        if idx > numel(newZacc)
            error('ASC 文件中含有的 IMU 行数大于补偿数据行数！');
        end

        % 00000077 后必须有 6 个字段：Za, -Ya, Xa, Zg, -Yg, Xg（最后一个带 *checksum）
        if idx77 + 6 > numel(parts)
            error('第 %d 行格式异常：00000077 后字段不足 6 个。', idx);
        end

        % 用新加速度数据替换前3个数值（角速度保持不变）
        Za  = newZacc(idx);
        mYa = newMinusYacc(idx);
        Xa  = newXacc(idx);

        % 格式化加速度数字为字符串
        accStr = { ...
            sprintf('%d', Za), ...
            sprintf('%d', mYa), ...
            sprintf('%d', Xa) };

        % 只替换前3个字段（加速度）
        for k = 1:3
            parts(idx77 + k) = string(accStr{k});
        end

        % 角速度字段（第4、5、6个字段）保持不变，不需要修改

        % 计数 +1，对应下一行补偿数据
        idx = idx + 1;

        % 重新组装成一整行
        tline = char(join(parts, ','));
    end

    fprintf(fid_out, '%s\n', tline);
end

fclose(fid_in);
fclose(fid_out);

fprintf('处理完成，生成文件：%s（只替换了加速度数据）\n', asc_out);

