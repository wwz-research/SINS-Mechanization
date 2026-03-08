/**
* @file     Calibration.h
* @brief    标定相关结构体和函数声明
* @details  本文件定义了加速度计和陀螺仪标定相关的结构体和函数接口，包括六位置法标定加速度计、\n
*           角位置法标定陀螺仪以及误差补偿等功能。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/01
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/01, Weizhen Wang, Create
*/


#pragma once

#include <fstream>   // ifstream，ofstream
#include <iostream>   // cout，endl
#include <iomanip>    // fixed，setprecision

#include"baseSDC.h"
#include"ReadFile.h"
#include"MatrixAndVect.h"
using namespace std;

/**
* @brief    加速度计六位置标定结构体
*
* 用于存储加速度计标定过程中的比力平均值、零偏、标度因子、交叉耦合以及标定矩阵等信息。
*/
struct AccCaliError
{
    double bilimean[18];    // 实际测得的比力的平均值3*6      
    double bias[3];         // 零偏XYZ
    double scale[3];        // 比例因子误差XYZ
    double cross[6];        // 交轴耦合误差
    double calibration[12]; // 加速度计误差矩阵3*4

    AccCaliError()
    {
        for (int i = 0; i < 18; ++i) bilimean[i] = 0.0;
        for (int i = 0; i < 3; ++i) bias[i] = 0.0;
        for (int i = 0; i < 3; ++i) scale[i] = 0.0;
        for (int i = 0; i < 6; ++i) cross[i] = 0.0;
        for (int i = 0; i < 12; ++i) calibration[i] = 0.0;
    }

    /**
    * @brief    从标定矩阵中提取零偏
    */
    void SetBias() { for (int i = 0; i < 3; i++) { bias[i] = calibration[i * 4 + 3]; } }

    /**
    * @brief    设置标度因子
    */
    void SetScale() { for (int i = 0; i < 3; i++) { scale[i] = calibration[i * 4 + i]- 1; } }

    /**
    * @brief    设置交叉耦合误差
    */
    void SetCross()
    {
        for (int i = 0; i < 2; i++)
        {
            cross[0] = calibration[1];    // x对y
            cross[1] = calibration[2];    // x对z
            cross[2] = calibration[4];    // y对x
            cross[3] = calibration[6];    // y对z
            cross[4] = calibration[8];    // z对x
            cross[5] = calibration[9];    // z对y
        }
    }
};

/**
* @brief    陀螺仪角位置标定结构体
*
* 用于存储陀螺仪标定过程中的零偏和标度因子误差。
*/
struct GyroCaliError
{
    double bias[3];           // 零偏XYZ(度/h)
    double scale[3];          // 比例因子XYZ

    GyroCaliError()
    {
        for (int i = 0; i < 3; i++)
        {
            bias[i] = scale[i] = 0.0;
        }
    }
};

/**
* @brief       计算标定位置比力平均值
* @param[in]   File    ifstream&        输入文件流
* @param[in]   Place   const int        标定位置，x_up x_down y_up y_down z_up z_down 分别为 0 1 2 3 4 5
* @param[out]  Error   AccCaliError&    加速度计标定误差结构体
*/
void CalCaliBillMean(ifstream& File, const int Place, AccCaliError& Error);

/**
* @brief       加速度计六位置标定
* @param[in,out] Error      AccCaliError&   加速度计标定误差结构体，输入比力平均值，输出标定矩阵
* @param[out]  OutputFile  ofstream&       输出文件流
*/
void CalAcc(AccCaliError& Error, ofstream& OutputFile);

/**
* @brief       补偿加速度计误差
* @param[in]   File        ifstream&           输入文件流
* @param[in]   Error       const AccCaliError& 加速度计标定误差结构体
* @param[out]  OutputFile  ofstream&           输出文件流
*/
void CompenAcc(ifstream& File, const AccCaliError& Error, ofstream& OutputFile);

/**
* @brief       利用加速度计六位置法中采集的静态陀螺数据标定陀螺零偏
* @param[in]   File_up     ifstream&           轴朝上的输入文件流
* @param[in]   File_down   ifstream&           轴朝下的输入文件流
* @param[in]   place       const int           标定轴，x、y、z分别为0、1、2
* @param[out]  Error       GyroCaliError&      陀螺仪标定误差结构体
*/
void CalGyrBias(ifstream& File_up, ifstream& File_down, const int place,GyroCaliError& Error);

/**
* @brief       对角速度进行积分对标定数据标度因子
* @param[in]   File_pos    ifstream&           正向旋转的输入文件流
* @param[in]   File_neg    ifstream&           反向旋转的输入文件流
* @param[in]   place       const int           标定轴，x、y、z分别为0、1、2
* @param[out]  Error       GyroCaliError&      陀螺仪标定误差结构体
*/
void CalGyrScale(ifstream& File_pos, ifstream& File_neg, const int place, GyroCaliError& Error);

/**
* @brief       补偿陀螺仪误差
* @param[in]   File        ifstream&               输入文件流
* @param[in]   place       const int               标定轴，x、y、z分别为0、1、2
* @param[in]   Error       const GyroCaliError&    陀螺仪标定误差结构体
* @param[out]  OutputFile  ofstream&               输出文件流
*/
void CompenGyro(ifstream& File, const int place, const GyroCaliError& Error, ofstream& OutputFile);
