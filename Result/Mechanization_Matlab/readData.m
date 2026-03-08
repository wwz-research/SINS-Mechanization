function data = readData(filename)
% 读取数据文件
% 输入参数：
%   filename - 数据文件路径
% 输出参数：
%   data - 数据table，列分别为：GPS周秒、纬度、经度、高、北向速度、东向速度、垂向速度、横滚角、俯仰角、航向角

    % 定义列名（英文）
    varNames = {'GPSTime', 'Latitude', 'Longitude', 'Height', 'Velocity_N', 'Velocity_E', ...
                'Velocity_D', 'Roll', 'Pitch', 'Yaw'};
    
    % 使用readtable读取数据，提高读取效率
    % 先读取数据，然后设置列名
    data = readtable(filename, 'Delimiter', ' ', 'MultipleDelimsAsOne', true, ...
                     'ReadVariableNames', false);
    
    % 设置列名
    data.Properties.VariableNames = varNames;
    
    % 验证数据维度
    if width(data) ~= 10
        error('数据文件格式错误：每行应包含10列数据');
    end
end

