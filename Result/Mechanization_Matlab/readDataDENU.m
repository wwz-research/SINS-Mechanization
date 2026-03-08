function data = readDataDENU(filename)
% 读取轨迹数据文件（ENU坐标系）
% 输入参数：
%   filename - 数据文件路径
% 输出参数：
%   data - 数据table，列分别为：GPS周秒、解算E、解算N、解算U、真值E、真值N、真值U

    % 定义列名（英文）
    varNames = {'GPSTime', 'E_Result', 'N_Result', 'U_Result', 'E_Ref', 'N_Ref', 'U_Ref'};
    
    % 使用readtable读取数据
    data = readtable(filename, 'Delimiter', ' ', 'MultipleDelimsAsOne', true, ...
                     'ReadVariableNames', false);
    
    % 设置列名
    numCols = width(data);
    if numCols == 7
        data.Properties.VariableNames = varNames;
    else
        error('数据文件格式错误：result_denu.txt应包含7列数据，实际为%d列', numCols);
    end
end

