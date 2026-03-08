/**
* @file     Align.h
* @brief    粗对准相关结构体和函数声明
* @details  本文件定义了粗对准相关的结构体和函数接口，包括姿态角计算、粗对准算法等。\n
*           提供了整段数据粗对准、每秒粗对准、每个历元粗对准以及时间噪声计算等功能。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/02
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/02, Weizhen Wang, Create
*           2025/12/26，Weizhen Wang，添加函数声明 CoarseAlign_Mechanization(ifstream& File, ALIGNPOS& Pos, ofstream& OutputFile)用于进行解析粗对准作为小推车惯导机械编排的初始姿态角
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
* @brief    粗对准结构体
*
* 用于存储粗对准过程中的姿态角、比力平均值、角速度平均值以及姿态矩阵等信息。
*/
struct ALIGNPOS
{
    GPSTIME time;       // 记录时间
    GPSTIME prevtime;   // 上一个历元的时间
    double billimean[3];     // 比力平均值
    double anglevelmean[3]; // 角速度平均值
    double yaw;     // 俯仰角 deg
    double pitch;   // 航向角 deg
    double roll;    // 横滚角 deg
    double biasA;   // 零偏主导的寻北误差
    double noiseA;  // 白噪声主导的寻北误差
    double Cbn[9];   // 姿态矩阵

    ALIGNPOS()
    {
        yaw = pitch = roll = 0.0; biasA = 0.0; noiseA = 0.0;
        for (int i = 0; i < 3; i++)
        {
            billimean[i] = 0.0; anglevelmean[i] = 0.0;
        }
        for (int j = 0; j < 9; j++) { Cbn[j] = 0.0; }
    }

    /**
    * @brief    从姿态矩阵中提取姿态角
    *
    * 根据姿态矩阵Cbn计算航向角、俯仰角和横滚角。
    */
    void SetPos()
    {
        pitch = atan(-Cbn[6] / sqrt(Cbn[7] * Cbn[7] + Cbn[8] * Cbn[8])) * Deg; // 俯仰角(度)
        yaw = atan2(Cbn[3], Cbn[0]) * Deg;    // 航向角(度)
        roll = atan2(Cbn[7], Cbn[8]) * Deg;    // 横滚角(度)
    }
};

/**
* @brief       计算姿态矩阵
* @param[in,out] Pos   ALIGNPOS&  粗对准位置结构体，输入比力和角速度平均值，输出姿态矩阵和姿态角
*/
void CalCbn(ALIGNPOS& Pos);

/**
* @brief       每个历元粗对准
* @param[in]   File        ifstream&   输入文件流
* @param[in,out] Pos1      ALIGNPOS&   当前历元的粗对准位置结构体
* @param[in]   Pos2        ALIGNPOS&   参考粗对准位置结构体
* @param[out]  OutputFile  ofstream&   输出文件流
*/
void CoarseAlign_EveryEpoch(ifstream& File, ALIGNPOS& Pos1,const ALIGNPOS& Pos2, ofstream& OutputFile);

/**
* @brief       每秒粗对准
* @param[in]   File        ifstream&   输入文件流
* @param[in,out] Pos1      ALIGNPOS&   当前秒的粗对准位置结构体
* @param[in]   Pos2        ALIGNPOS&   参考粗对准位置结构体
* @param[out]  OutputFile  ofstream&   输出文件流
*/
void CoarseAlign_EverySecond(ifstream& File, ALIGNPOS& Pos1,const ALIGNPOS& Pos2, ofstream& OutputFile);

/**
* @brief       整段粗对准
* @param[in]   File        ifstream&   输入文件流
* @param[out]  Pos         ALIGNPOS&   粗对准位置结构体
* @param[out]  OutputFile  ofstream&   输出文件流
*/
void CoarseAlign_Whole(ifstream& File, ALIGNPOS& Pos, ofstream& OutputFile);
void CoarseAlign_Mechanization(ifstream& File, ALIGNPOS& Pos, ofstream& OutputFile);//用于小推车静态数据的解析粗对准


/**
* @brief       计算白噪声随时间变化
* @param[in]   File        ifstream&   输入文件流
* @param[out]  OutputFile  ofstream&   输出文件流
*/
void CalTimeNoise(ifstream& File, ofstream& OutputFile);