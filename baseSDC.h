/**
* @file     baseSDC.h
* @brief    基础数据结构和常量定义
* @details  该文件包含了SINS系统中使用的基础数据结构、常量定义和宏定义，包括GPS时间、\n
*           原始数据、位置、速度、姿态、四元数等结构体，以及WGS84椭球参数、设备参数等常量。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/01
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/01, Weizhen Wang, Create
*           2025/12/21，Weizhen Wang, 增加有关机械编排的结构体和常量定义
*/

#pragma once
#include<cmath>
#include <iostream>   // cout，endl
using namespace std;

/** @brief  常数 */
#define PAI 3.141592653589      // 圆周率
#define Deg 180.0/PAI           // 弧度转化为度
#define Rad PAI/180.0           // 度转化为弧度
#define circle 360.0            // 一圈360度（deg）

/** @brief  WGS84椭球参数 */
#define we 7.292115e-5          // 地球自转角速度(rad/s)
#define gravity 9.7936174       // 重力加速度9.7936174
#define R_WGS84  6378137.0               //长半轴
#define F_WGS84  1.0/298.257223563       //扁率
#define E2_WGS84 0.0066943799901413165   // 第一偏心率平方


/** @brief  标定和粗对准：数据转化比力因子，惯导采样率 */
#define AS_Calibraton 1.5258789063e-6 // 标定：  XW-GI7681 加速度计数据转换比例因子
#define GS_Calibraton 1.0850694444e-7 // 标定：  XW-GI7681 陀螺仪数据转换比例因子
#define AS_Align 2.0e-8           // 粗对准：NoVATel SPAN-100C 加速度计数据转换比例因子
#define GS_Align 1.0e-9           // 粗对准：NoVATel SPAN-100C 陀螺仪数据转换比例因子
#define r_Calibraton 100         // 标定：  XW-GI7681 采样率100Hz
#define r_Align 200               // 粗对准：NoVATel SPAN-100C 采样率200Hz
#define fai 30.531651244 * Rad  // 转台实验室纬度(rad)

/** @brief  惯导机械编排：数据转化比力因子，惯导采样率 */
#define AS_Mechanization 1.5258789063e-6 // 小推车：  XW-GI7681 加速度计数据转换比例因子
#define GS_Mechanization 1.0850694444e-7 // 小推车：  XW-GI7681 陀螺仪数据转换比例因子
#define r_Mechanization 100              // 小推车：  XW-GI7681 采样率100Hz

/** @brief  惯导机械编排：示例数据初始值 */
const double initial_time = 91620.0;                    // 初始时间 (秒)
const double initial_pos[3] = { 23.1373950708 * Rad ,113.3713651222 * Rad ,2.175 };// 单位：rad,rad,m
const double initial_vel[3] = { 0.0,0.0,0.0 };// 速度，单位：m/s
const double initial_atti[3] = { 0.0107951084511778 * Rad ,-2.14251290749072 * Rad ,-75.7498049314083 * Rad };//  横滚角、俯仰角、航向角，单位：Rad

/** @brief  惯导机械编排：小推车数据初始值（位置、姿态采用初始静态时段参考真值的平均值，速度为0） */
const double ours_initial_time = 98129.000;                    // 初始时间 (秒)
const double ours_initial_pos[3] = { 30.5278839868 * Rad ,114.3557289938 * Rad ,19.702 };// 单位：rad,rad,m
const double ours_initial_vel[3] = { 0.0,0.0,0.0 };// 速度，单位：m/s
const double ours_initial_atti[3] = { 0.61094776175 * Rad ,0.2325664338 * Rad ,5.0291456411 * Rad };//  横滚角、俯仰角、航向角，单位：Rad


const int zero_time_intervals_num = 5; // 零速修正区间个数


/**
* @brief    设备类型枚举
*
* 定义不同的设备类型，用于区分标定和粗对准所使用的设备。
*/
enum DeviceType { Calibraton, Align, Mechanization};      // XWGI为标定(calibration) 设备  NSC为粗对准(align)设备

/**
* @brief    GPS时间结构体
*
* 用于表示GPS时间信息，包含GPS周和GPS周秒两个成员。
*/
struct GPSTIME
{
    int Week;       // GPS周
    double Second;  // GPS周秒

    GPSTIME() { Week = 0; Second = 0; }
};

/**
* @brief    加速度计数据结构体
*
* 用于存储加速度计的三维数据，包含X、Y、Z三个轴的加速度信息。
*/
struct ACCDAT
{
    double X, Y, Z;  // 加速度计X、Y、Z轴数据

    ACCDAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};

/**
* @brief    陀螺仪数据结构体
*
* 用于存储陀螺仪的三维数据，包含X、Y、Z三个轴的角速度信息。
*/
struct GYRODAT
{
    double X, Y, Z;  // 陀螺仪X、Y、Z轴数据

    GYRODAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};

/**
* @brief    原始数据结构体
*
* 用于整合原始数据信息，包含GPS时间、加速度计数据和陀螺仪数据。
*/

struct RAWDAT
{
    GPSTIME time;   // 周 秒
    ACCDAT acc;     // 加速度计数据：加速度(m/s/s)
    ACCDAT acc_v;   // 加速度计数据：速度增量(m/s)
    GYRODAT gyr_v;  // 陀螺仪数据：角速度增量（rad/s）
    GYRODAT gyr;    // 陀螺仪数据：角度增量（rad）
};


/**
* @brief   位置结构体
*
* 用于存储位置信息，包括纬度、经度和高度。
*/
struct POSITION
{
    double latitude;    // 纬度,rad
    double longitude;   // 经度,rad
    double H;           // 高程
    POSITION() { latitude = longitude = H = 0.0; }
};

/**
* @brief   轨迹结构体
*
* 用于存储ENU坐标系下的轨迹信息。
*/
struct dENU
{
    double dE;
    double dN;
    double dU;
    dENU() { dE = dN = dU = 0.0; }
};

/**
* @brief   速度结构体
*
* 用于存储导航系下的速度信息。
*/
struct VELOCITY
{
    double Vn, Ve, Vd;  // 北东地速度
    VELOCITY() { Vn = Ve = Vd = 0.0; }
};

/**
* @brief   姿态结构体
*
* 用于存储姿态角信息，包括横滚角、俯仰角和航向角。
*/
struct ATTITUDE
{
    double roll, pitch, yaw;    // 横滚、俯仰、航向角
    ATTITUDE() { roll = pitch = yaw = 0.0; }
};

/**
* @brief   姿态四元数结构体
*
* 用于存储和计算姿态四元数，包括载体系和导航系的四元数增量计算。
*/
struct QUATER
{
    double q[4];    // q0 q1 q2 q3
    QUATER() { for (int i = 0; i < 4; i++) { q[i] = 0.0; } }


    //计算b系四元数q
    void CalQ_b(double Fai_k[3])
    {
        double norm = sqrt(pow(Fai_k[0] / 2.0, 2) + pow(Fai_k[1] / 2.0, 2) + pow(Fai_k[2] / 2.0, 2));
        double coef = 0.0;
        if(norm!=0.0) coef = sin(norm) / norm;// 后三项的系数，注意norm！=0.0
        q[0] = cos(norm);
        q[1] = 0.5 * Fai_k[0] * coef;
        q[2] = 0.5 * Fai_k[1] * coef;
        q[3] = 0.5 * Fai_k[2] * coef;
    }

    //计算n系四元数q
    void CalQ_n(double Theta_k[3])
    {
        double norm = sqrt(pow(Theta_k[0] / 2.0, 2) + pow(Theta_k[1] / 2.0, 2) + pow(Theta_k[2] / 2.0, 2));
        double coef = 0.0;
        if (norm != 0.0) coef = -sin(norm) / norm;// 后三项的系数，注意norm！=0.0
        q[0] = cos(norm);
        q[1] = 0.5 * Theta_k[0] * coef;
        q[2] = 0.5 * Theta_k[1] * coef;
        q[3] = 0.5 * Theta_k[2] * coef;
    }

    void SetQbn(ATTITUDE atti)    // 设置初始姿态四元数
    {
        double Fai = atti.roll / 2.0, theta = atti.pitch / 2.0, pusai = atti.yaw / 2.0;
        q[0] = cos(Fai) * cos(theta) * cos(pusai) + sin(Fai) * sin(theta) * sin(pusai);
        q[1] = sin(Fai) * cos(theta) * cos(pusai) - cos(Fai) * sin(theta) * sin(pusai);
        q[2] = cos(Fai) * sin(theta) * cos(pusai) + sin(Fai) * cos(theta) * sin(pusai);
        q[3] = cos(Fai) * cos(theta) * sin(pusai) - sin(Fai) * sin(theta) * cos(pusai);
    }

};

/**
* @brief   方向余弦矩阵结构体
*
* 用于存储和计算从载体系到导航系的旋转矩阵Cbn。
*/
struct CosineMatrix
{
    double DCM[9];
    CosineMatrix() { for (int i = 0; i < 9; i++) { DCM[i] = 0.0; } }

    void SetCbn(ATTITUDE atti)   // 设置Cbn余弦矩阵
    {
        double theta = atti.pitch, pusai = atti.yaw, Fai = atti.roll;  // 俯仰角、航向角、横滚角(rad)
        DCM[0] = cos(theta) * cos(pusai);
        DCM[1] = -cos(Fai) * sin(pusai) + sin(Fai) * sin(theta) * cos(pusai);
        DCM[2] = sin(Fai) * sin(pusai) + cos(Fai) * sin(theta) * cos(pusai);
        DCM[3] = cos(theta) * sin(pusai);
        DCM[4] = cos(Fai) * cos(pusai) + sin(Fai) * sin(theta) * sin(pusai);
        DCM[5] = -sin(Fai) * cos(pusai) + cos(Fai) * sin(theta) * sin(pusai);
        DCM[6] = -sin(theta);
        DCM[7] = sin(Fai) * cos(theta);
        DCM[8] = cos(Fai) * cos(theta);
    }
};

/**
* @brief   单历元惯导状态结构体
*
* 用于存储单个历元的惯导状态信息，包括时间、位置、轨迹、速度、姿态和四元数。
*/
struct InsEpochState
{
    GPSTIME time;   // 周 秒
    POSITION pos;   // 位置结果 rad,rad,m
    dENU denu;      // 轨迹
    VELOCITY vel;   // 速度结果
    ATTITUDE atti;  // 姿态结果 rad,rad,rad
    QUATER quater;  // 姿态四元数
};

/**
* @brief   多历元惯导状态结构体
*
* 用于存储当前历元和前两个历元的惯导状态，用于机械编排计算。
*/
struct InsState
{
    InsEpochState cur;      // 当前历元状态
    InsEpochState prev1;    // 前一个历元状态
    InsEpochState prev2;    // 前两个历元状态
};

/**
* @brief   时间间隔结构体
*
* 用于存储时间间隔信息，包括开始时间、结束时间和是否使用标志。
*/
struct TimeInterval
{
    double start;   // 开始静止时刻
    double end;     // 开始启动时刻
    bool used;      // 是否使用过

    TimeInterval(double starttime = 0.0, double endtime = 0.0, bool isUsed = false)
        :start(starttime), end(endtime), used(isUsed) { }
};

/**
* @brief   时间间隔数组结构体
*
* 用于存储多个时间间隔，主要用于零速度时间段的管理。
*/
struct TimeIntervalsArray
{
    // 总共十四个需要零速修正的区间
    TimeInterval interval[zero_time_intervals_num]; // interval[14]

    // 初始化时间区间
    TimeIntervalsArray()
    {
        //interval[0] = TimeInterval(98129.000, 98568.000, false);
        interval[0] = TimeInterval(98822.000, 98885.000, false);
        interval[1] = TimeInterval(99196.000, 99282.000, false);
        interval[2] = TimeInterval(99531.500, 99593.000, false);
        interval[3] = TimeInterval(99849.400, 99920.000, false);
        interval[4] = TimeInterval(100190.000, 100520.260, false);
        //interval[0] = TimeInterval(98129.000, 98575.000, false);
        //interval[1] = TimeInterval(98820.000, 98892.000, false);
        //interval[2] = TimeInterval(99196.000, 99285.000, false);
        //interval[3] = TimeInterval(99529.000, 99596.000, false);
        //interval[4] = TimeInterval(99852.000, 99920.000, false);
        //interval[5] = TimeInterval(100188.000, 100520.260, false);

    }

    // 获取指定索引的时间区间
    TimeInterval& getInterval(int index)
    {
        if (index >= 0 && index < 5)return interval[index];
        else {
            cout << "Index out of our time interval bounds!\n";
        }
    }
};