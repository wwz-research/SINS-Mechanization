% 读取数据的函数
function data = ReadData(filename, mode)
    switch(mode)
        case 0
            % 打开文件
            fileID = fopen(filename, 'r');
            if fileID == -1
                error('File could not be opened.');
            end

            % 读取数据：每行数据包括时间、加速度、角速度等
            data = [];
            while ~feof(fileID)
                line = fgetl(fileID);  % 读取一行
        
                % 如果行以 '%' 开头，则跳过
                if startsWith(line, '%')
                    continue;
                end
        
                % 将每行数据分割并存储
                if ischar(line)
                    linedata = str2double(strsplit(line, ','));  % 使用逗号分割
                end
                if ~any(isnan(linedata))
                    data = [data;linedata];
                end
            end
        
            % 关闭文件
            fclose(fileID);

        case 1
        % 打开文件
        fileID = fopen(filename, 'r');
        if fileID == -1
            error('无法打开文件');
        end
    
        % 使用 textscan 读取文件数据
        data = textscan(fileID, '%f %f %f %f', 'Delimiter', ',', 'HeaderLines', 1);
        
        % 关闭文件
        fclose(fileID);
        
        % 将数据转为 table，列名可以根据需要进行修改
        data = table(data{1}, data{2}, data{3}, data{4});

        case 2
        fileID = fopen(filename, 'r');
        if fileID == -1
            error('无法打开文件');
        end
    
        % 使用 textscan 读取文件数据
        data = textscan(fileID, '%f %f', 'Delimiter', ',', 'HeaderLines', 2);
        
        % 关闭文件
        fclose(fileID);
        
        % 将数据转为 table，列名可以根据需要进行修改
        data = table(data{1}, data{2});

    end
end
