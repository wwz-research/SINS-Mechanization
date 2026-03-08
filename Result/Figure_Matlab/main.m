% 提示用户输入
choice = input('请选择: 0加速度计标定绘图; 1陀螺仪标定绘图; 2对准绘图：');
if(~(choice == 0 || choice == 1 || choice == 2))
    fprintf("\n无法识别：%d 程序结束!\n",choice);
end

% 选择case
switch choice
    % 绘制加速度计的原始数据与补偿数据图像
    case 0
        AccCaliX_up = 'AccCali/x_up_Raw_Com.txt';  % 加速度计原始数据与补偿数据
        AccCaliX_down = 'AccCali/x_down_Raw_Com.txt';
        AccCaliY_up = 'AccCali/y_up_Raw_Com.txt';
        AccCaliY_down = 'AccCali/y_down_Raw_Com.txt';
        AccCaliZ_up = 'AccCali/z_up_Raw_Com.txt';
        AccCaliZ_down = 'AccCali/z_down_Raw_Com.txt';

        dAccCaliX_up = ReadData(AccCaliX_up, 0);  % ReadData函数读取数据
        dAccCaliX_down = ReadData(AccCaliX_down, 0);
        dAccCaliY_up = ReadData(AccCaliY_up, 0);
        dAccCaliY_down = ReadData(AccCaliY_down, 0);
        dAccCaliZ_up = ReadData(AccCaliZ_up, 0);
        dAccCaliZ_down = ReadData(AccCaliZ_down, 0);

        AccCali_fig(dAccCaliX_up, 0, 0);  % 绘制图像
        AccCali_fig(dAccCaliX_down, 0, 1);
        AccCali_fig(dAccCaliY_up, 1, 0);
        AccCali_fig(dAccCaliY_down, 1, 1);
        AccCali_fig(dAccCaliZ_up, 2, 0);
        AccCali_fig(dAccCaliZ_down, 2, 1);
    
    % 绘制陀螺仪原始数据与补偿数据图像
    case 1
        GyrCaliX_pos = 'GyrCali/x+360_Raw_Com.txt';  % 陀螺仪原始数据与补偿数据
        GyrCaliX_neg = 'GyrCali/x-360_Raw_Com.txt';
        GyrCaliY_pos = 'GyrCali/y+360_Raw_Com.txt';
        GyrCaliY_neg = 'GyrCali/y-360_Raw_Com.txt';
        GyrCaliZ_pos = 'GyrCali/z+360_Raw_Com.txt';
        GyrCaliZ_neg = 'GyrCali/z-360_Raw_Com.txt';
 
        dGyrCaliX_pos = ReadData(GyrCaliX_pos, 0);  % ReadData函数读取数据
        dGyrCaliX_neg = ReadData(GyrCaliX_neg, 0);
        dGyrCaliY_pos = ReadData(GyrCaliY_pos, 0);
        dGyrCaliY_neg = ReadData(GyrCaliY_neg, 0);
        dGyrCaliZ_pos = ReadData(GyrCaliZ_pos, 0);
        dGyrCaliZ_neg = ReadData(GyrCaliZ_neg, 0);

        GyrCali_fig(dGyrCaliX_pos, 1); % 绘制图像
        GyrCali_fig(dGyrCaliX_neg, 0);
        GyrCali_fig(dGyrCaliY_pos, 0);
        GyrCali_fig(dGyrCaliY_neg, 1);
        GyrCali_fig(dGyrCaliZ_pos, 1);
        GyrCali_fig(dGyrCaliZ_neg, 0);

    % 绘制 整段数据平均 / 每秒平均 / 每历元的对准结果
    case 2
        Align_whole = 'Align/Align_Whole.txt';  % 整段数据
        Align_Second = 'Align/Align_Second.txt';    % 每秒数据
        Align_Epoch = 'Align/Align_Epoch.txt';  % 每历元数据
        Align_Noise = 'Align/Align_Deviation.txt';  % 白噪声偏差

        dAlign_whole = ReadData(Align_whole, 0);  % ReadData函数读取数据
        dAlign_second = ReadData(Align_Second, 0);
        dAlign_epoch = ReadData(Align_Epoch, 1);
        dAlign_noise = ReadData(Align_Noise, 2);

        Align_fig(dAlign_whole, dAlign_second, dAlign_epoch, 0);   % 绘制图像
        Align_fig(dAlign_noise, dAlign_noise, dAlign_noise, 1);

end