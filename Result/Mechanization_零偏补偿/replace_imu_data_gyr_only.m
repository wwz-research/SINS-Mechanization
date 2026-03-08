%% 参数设置 - 只替换角速度数据
gyr_sf = 1.0850694444e-7;   % 陀螺仪数据转换比例因子
fs     = 100;               % 采样率

gyr_file = 'group5_gyrCali.txt';
asc_in   = 'group5.ASC';
asc_out  = 'group5_new_gyr.ASC';

%% 读取补偿后的角速度
% group5_gyrCali.txt: [time, Gx, Gy, Gz]
gyrData = readmatrix(gyr_file);
Gx = gyrData(:,2);
Gy = gyrData(:,3);
Gz = gyrData(:,4);

%% 物理量 -> 计数（注意是"除以"比例因子和采样率）
gyrX_cnt = round( Gx ./ (gyr_sf * fs) );
gyrY_cnt = round( Gy ./ (gyr_sf * fs) );
gyrZ_cnt = round( Gz ./ (gyr_sf * fs) );

%% 坐标轴顺序与 Y 轴符号
% 原始格式：00000077 后依次为
%   Z轴加速度、-Y轴加速度、X轴加速度、Z轴角速度、-Y轴角速度、X轴角速度
newZgyr   = gyrZ_cnt;        % Z
newMinusYgyr = -gyrY_cnt;    % -Y
newXgyr   = gyrX_cnt;        % X

%% 按行读写 ASC 文件，只替换 00000077 后的后3个数值（角速度）
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
        if idx > numel(newZgyr)
            error('ASC 文件中含有的 IMU 行数大于补偿数据行数！');
        end

        % 00000077 后必须有 6 个字段：Za, -Ya, Xa, Zg, -Yg, Xg（最后一个带 *checksum）
        if idx77 + 6 > numel(parts)
            error('第 %d 行格式异常：00000077 后字段不足 6 个。', idx);
        end

        % 取原来第 6 个字段（最后一个）以保留 * 后的校验和字符串
        lastTok = char(parts(idx77+6));
        starPos = strfind(lastTok, '*');
        if ~isempty(starPos)
            oldChecksum = lastTok(starPos+1:end);   % 原校验和字符串
        else
            oldChecksum = '';
        end

        % 用新角速度数据替换后3个数值（加速度保持不变）
        Zg  = newZgyr(idx);
        mYg = newMinusYgyr(idx);
        Xg  = newXgyr(idx);

        % 格式化角速度数字为字符串
        gyrStr = { ...
            sprintf('%d', Zg), ...
            sprintf('%d', mYg), ...
            sprintf('%d', Xg) };

        % 加速度字段（第1、2、3个字段）保持不变，不需要修改
        
        % 替换第4、5个字段（角速度的前两个）
        for k = 1:2
            parts(idx77 + 3 + k) = string(gyrStr{k});
        end

        % 第 6 个字段（最后一个角速度）加回原来的 *checksum
        if ~isempty(starPos)
            parts(idx77 + 6) = string(sprintf('%s*%s', gyrStr{3}, oldChecksum));
        else
            parts(idx77 + 6) = string(gyrStr{3});
        end

        % 计数 +1，对应下一行补偿数据
        idx = idx + 1;

        % 重新组装成一整行
        tline = char(join(parts, ','));
    end

    fprintf(fid_out, '%s\n', tline);
end

fclose(fid_in);
fclose(fid_out);

fprintf('处理完成，生成文件：%s（只替换了角速度数据）\n', asc_out);

