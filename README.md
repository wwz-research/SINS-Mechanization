# SINS-Mechanization
基于 C++ 的捷联惯导（SINS）课程实践项目，实现 IMU 标定、解析粗对准以及示例与小推车数据的惯导机械编排，并提供 Matlab 与 Web 动态可视化工具。

项目实验说明与结果分析见 [项目实验报告.pdf](项目实验报告.pdf)。

## 功能概述

程序提供五种工作模式，通过控制台交互选择：

| 模式 | 功能 |
|------|------|
| 0 | 加速度计标定（六位置法） |
| 1 | 陀螺仪标定（角位置法） |
| 2 | 粗对准（解析粗对准，含整段/每秒/每历元姿态及白噪声分析） |
| 3 | 示例数据机械编排（与参考真值比较） |
| 4 | 小推车数据机械编排（支持零速修正） |

## 环境与构建

- **语言**：C++
- **依赖**：仅使用 C++ 标准库，无第三方库
- **IDE**：Visual Studio  2022
- **平台**：x64

## 项目结构

### 顶层目录说明

| 目录 | 说明 |
|------|------|
| **Data/** | **需要使用的数据**。程序从本目录读取标定、粗对准及机械编排所需的 IMU 与参考数据，运行前需按下方「目录与数据要求」准备好相应文件。 |
| **Result/** | **解算结果及 Matlab 绘图函数**。含三部分：`Figure_Matlab/`（标定、粗对准结果及绘图脚本）、`Mechanization_Matlab/`（机械编排结果及绘图/对比脚本）、`Mechanization_零偏补偿/`（零偏补偿中间数据及替换脚本）。程序将数值结果写入前两个目录，Matlab 脚本用于绘图与对比分析。 |
| **tools/web_viewers/** | **机械编排结果的动态可视化**。通过 `Drawing.html` 与 `data_loader.js` 在浏览器中对机械编排输出的轨迹等结果进行动态展示。 |

### 源代码文件

| 文件 | 说明 |
|------|------|
| `SINS_zhen.cpp` | 主程序入口，模式分发与流程控制 |
| `baseSDC.h` | 公共数据结构与常量（WGS84、采样率、初始值等） |
| `ReadFile.h/cpp` | IMU 与参考数据读取与解析 |
| `Calibration.h/cpp` | 加速度计/陀螺仪标定 |
| `Align.h/cpp` | 粗对准 |
| `SINSMechanization.h/cpp` | 机械编排（姿态/速度/位置更新） |
| `CmnFun.cpp` | 四元数、重力、外推等辅助函数 |
| `CoordinateTransformation.h/cpp` | 坐标转换与 ENU 误差计算 |
| `MatrixAndVect.h/cpp` | 矩阵与向量运算 |

## 目录与数据要求

程序使用相对路径，需在可执行文件同级或工作目录下建立以下结构。

### Data（输入数据）

```
Data/
├── Calibration/          # 标定数据（mode 0, 1）
│   ├── x_up.ASC, x_down.ASC
│   ├── y_up.ASC, y_down.ASC
│   ├── z_up.ASC, z_down.ASC
│   ├── x_tl_1.ASC, x_tl_2.ASC
│   ├── y_tl_1.ASC, y_tl_2.ASC
│   └── z_tl_1.ASC, z_tl_2.ASC
├── Align/                # 粗对准数据（mode 2）
│   └── Align_30min.ASC
├── MechanizationData/    # 小推车机械编排（mode 4）
│   ├── group5.ASC
│   └── IEproject5.ref
└── MechanizationExampleData/  # 示例机械编排（mode 3）
    ├── IMU.bin
    └── PureINS.bin
```

### Result（输出与 Matlab）

程序会自动写入解算结果；目录中另含 Matlab 绘图脚本，用于结果可视化与对比。

```
Result/
├── Figure_Matlab/           # 标定、粗对准结果及对应 .m 绘图脚本
│   ├── AccCali/             # 加速度计标定结果（AccCaliError.txt、各位置 Raw_Com.txt）
│   ├── GyrCali/             # 陀螺标定结果（GyroCalibration.txt、各轴 Raw_Com.txt）
│   ├── Align/               # 粗对准结果（Align_Whole.txt 等）
│   ├── main.m, ReadData.m
│   └── AccCali_fig.m, GyrCali_fig.m, Align_fig.m
├── Mechanization_Matlab/    # 机械编排结果及 .m 绘图/对比脚本
│   ├── main.m, readData.m, readDataDENU.m, readDataDiff.m
│   ├── plotAllError.m, compareError.m, compareCalibrationMethods.m, compareVchange.m
│   ├── plotTrajectory.m, plotComparison.m
│   └── Mechanization/       # 小推车机械编排（result.txt, result_ref.txt, result_diff.txt, result_denu.txt, result_Align.txt）
└── Mechanization_零偏补偿/  # 零偏补偿用中间数据及 .m 替换脚本
    ├── group5.ASC, group5_AccCali.txt, group5_gyrCali.txt
    ├── group5_new.ASC, group5_new_acc.ASC, group5_new_gyr.ASC
    └── replace_imu_data.m, replace_imu_data_acc_only.m, replace_imu_data_gyr_only.m
```

### 数据格式说明

- **ASC 标定/对准/小推车**：`%RAWIMUSA,周,周秒;...` 格式，逗号分隔，含加速度计与陀螺仪原始量
- **示例 IMU.bin**：二进制，每历元 7 个 double（时间戳、陀螺 XYZ、加速度 XYZ）
- **示例 PureINS.bin**：二进制，每历元 10 个 double（时间、位置、速度、姿态）
- **参考 IEproject5.ref**：文本，含 Week、GPSTime、经纬高、速度、姿态等字段

## 作者

Weizhen Wang (wwz131407080103@qq.com)

## 许可证

本项目采用 [MIT License](LICENSE) 开源。
