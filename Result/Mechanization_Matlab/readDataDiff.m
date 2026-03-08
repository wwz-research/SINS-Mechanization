function data = readDataDiff(filename)
% 读取差分结果数据文件（包含dE、dN、dU列）
% 输入参数：
%   filename - 数据文件路径
% 输出参数：
%   data - 数据table，列分别为：GPS周秒、纬度、经度、高、北向速度、东向速度、垂向速度、
%          横滚角、俯仰角、航向角、dE、dN、dU

    % 定义列名（英文），包含新增的dE、dN、dU列
    varNames = {'GPSTime', 'Latitude', 'Longitude', 'Height', 'Velocity_N', 'Velocity_E', ...
                'Velocity_D', 'Roll', 'Pitch', 'Yaw', 'dE', 'dN', 'dU'};
    
    % 使用readtable读取数据
    data = readtable(filename, 'Delimiter', ' ', 'MultipleDelimsAsOne', true, ...
                     'ReadVariableNames', false);
    
    % 设置列名
    numCols = width(data);
    if numCols == 13
        data.Properties.VariableNames = varNames;
    elseif numCols == 10
        % 兼容旧格式（10列）
        data.Properties.VariableNames = varNames(1:10);
    else
        error('数据文件格式错误：result_diff.txt应包含10或13列数据，实际为%d列', numCols);
    end
end

