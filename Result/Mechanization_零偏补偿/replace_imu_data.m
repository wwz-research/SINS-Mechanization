%% 参数设置
acc_sf = 1.5258789063e-6;   % 加速度计数据转换比例因子
gyr_sf = 1.0850694444e-7;   % 陀螺仪数据转换比例因子
fs     = 100;               % 采样率

acc_file = 'group5_AccCali.txt';
gyr_file = 'group5_gyrCali.txt';
asc_in   = 'group5.ASC';
asc_out  = 'group5_new.ASC';

%% 读取补偿后的加速度和角速度
% group5_AccCali.txt: [time, Ax, Ay, Az]
accData = readmatrix(acc_file);   % 默认按逗号分隔
Ax = accData(:,2);
Ay = accData(:,3);
Az = accData(:,4);

% group5_gyrCali.txt: [time, Gx, Gy, Gz]
gyrData = readmatrix(gyr_file);
Gx = gyrData(:,2);
Gy = gyrData(:,3);
Gz = gyrData(:,4);

% 简单一致性检查（可选）
if size(accData,1) ~= size(gyrData,1)
    error('加速度与陀螺补偿数据行数不一致！');
end

%% 物理量 -> 计数（注意是"除以"比例因子和采样率）
accX_cnt = round( Ax ./ (acc_sf * fs) );
accY_cnt = round( Ay ./ (acc_sf * fs) );
accZ_cnt = round( Az ./ (acc_sf * fs) );

gyrX_cnt = round( Gx ./ (gyr_sf * fs) );
gyrY_cnt = round( Gy ./ (gyr_sf * fs) );
gyrZ_cnt = round( Gz ./ (gyr_sf * fs) );

%% 坐标轴顺序与 Y 轴符号
% 原始格式：00000077 后依次为
%   Z轴加速度、-Y轴加速度、X轴加速度、Z轴角速度、-Y轴角速度、X轴角速度
newZacc   = accZ_cnt;        % Z
newMinusYacc = -accY_cnt;    % -Y
newXacc   = accX_cnt;        % X

newZgyr   = gyrZ_cnt;        % Z
newMinusYgyr = -gyrY_cnt;    % -Y
newXgyr   = gyrX_cnt;        % X

%% 按行读写 ASC 文件，替换 00000077 后的 6 个数值
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

        % 取原来第 6 个字段（最后一个）以保留 * 后的校验和字符串
        lastTok = char(parts(idx77+6));
        starPos = strfind(lastTok, '*');
        if ~isempty(starPos)
            oldChecksum = lastTok(starPos+1:end);   % 原校验和字符串
        else
            oldChecksum = '';
        end

        % 用新数据替换数值（但暂时保留原校验和）
        Za  = newZacc(idx);
        mYa = newMinusYacc(idx);
        Xa  = newXacc(idx);
        Zg  = newZgyr(idx);
        mYg = newMinusYgyr(idx);
        Xg  = newXgyr(idx);

        % 格式化每个数字为字符串
        numStr = { ...
            sprintf('%d', Za), ...
            sprintf('%d', mYa), ...
            sprintf('%d', Xa), ...
            sprintf('%d', Zg), ...
            sprintf('%d', mYg), ...
            sprintf('%d', Xg) };

        % 前 5 个字段是纯数字
        for k = 1:5
            parts(idx77 + k) = string(numStr{k});
        end

        % 第 6 个字段加回原来的 *checksum（只改数值，不改校验和本身）
        if ~isempty(starPos)
            parts(idx77 + 6) = string(sprintf('%s*%s', numStr{6}, oldChecksum));
        else
            parts(idx77 + 6) = string(numStr{6});
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

fprintf('处理完成，生成文件：%s\n', asc_out);

