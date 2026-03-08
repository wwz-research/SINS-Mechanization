/**
* @file     ReadFile.h
* @brief    IMU ASC文件读取与解析的相关数据结构体和函数声明
* @details  定义设备类型、GPS时间、加速度计数据、陀螺仪数据及原始数据等结构体，\n
*           声明读取ASV标定文件和对准文件的函数。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/01
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/01, Weizhen Wang, Create
*           2025/11/02, Weizhen Wang, 添加函数声明 ReadFile_Ali(ifstream&, DeviceType, RAWDAT&) 用于读取粗对准文件数据并写入 Rawdata 结构体。
*			2025/12/21, Weizhen Wang, 添加函数声明 ReadExamplePureIMUData(ifstream& file, RAWDAT& Rawdata) 用于读取示例数据
*			2025/12/21, Weizhen Wang, 添加函数声明 ReadExamplePureINSRef(ifstream& file, InsEpochState& InsRef) 用于读取示例数据的机械编排参考结果
*			2025/12/24, Weizhen Wang, 添加函数声明 ReadPureIMUData(ifstream& file, const DeviceType device, RAWDAT& Rawdata) 用于读取小推车的IMU数据
*			2025/12/24, Weizhen Wang, 添加函数声明 ReadPureINSRef(ifstream& file, InsEpochState& InsRef) 用于读取小推车的机械编排参考结果
*/

#pragma once

#include <fstream>   // ifstream
#include <string>    // string、stoi、stod
#include <sstream>   // stringstream

#include"baseSDC.h"
using namespace std;

/**
* @brief       读取标定文件数据
* @param[in]   file      ifstream&      已打开的IMU ASC文件流
* @param[in]   device    DeviceType     设备类型，必须为Calibraton时才有效
* @param[out]  Rawdata   RAWDAT&       读取一行解析后的IMU数据，包括时间、加速度计和陀螺仪
* @return      bool  返回是否读取并解析成功，true表示成功，false表示失败
*/
bool ReadFile_Cali(ifstream& file, const DeviceType device, RAWDAT& Rawdata); 

/**
* @brief       读取粗对准文件数据
* @param[in]   file      ifstream&      已打开的IMU ASC文件流
* @param[in]   device    DeviceType     设备类型，必须为Align时才有效
* @param[out]  Rawdata   RAWDAT&       读取一行解析后的原始IMU数据，包括时间、加速度计和陀螺仪
* @return      bool  返回是否读取并解析成功，true表示成功，false表示失败
*/
bool ReadFile_Ali(ifstream& file, const DeviceType device, RAWDAT& Rawdata);  

/**
* @brief       读取示例数据IMU数据
* @param[in]   file      ifstream&      已打开的二进制文件流
* @param[out]  Rawdata   RAWDAT&       读取一个历元的IMU数据
* @return      bool  返回是否读取成功，true表示成功，false表示失败
*/
bool ReadExamplePureIMUData(ifstream& file, RAWDAT& Rawdata);

/**
* @brief       读取示例数据机械编排参考结果
* @param[in]   file      ifstream&      已打开的二进制文件流
* @param[out]  InsRef    InsEpochState& 读取一个历元的机械编排参考结果
* @return      bool  返回是否读取成功，true表示成功，false表示失败
*/
bool ReadExamplePureINSRef(ifstream& file, InsEpochState& InsRef);

/**
* @brief       读取小推车IMU数据
* @param[in]   file      ifstream&      已打开的ASC文件流
* @param[in]   device    DeviceType     设备类型，必须为Mechanization时才有效
* @param[out]  Rawdata   RAWDAT&       读取一个历元的IMU数据
* @return      bool  返回是否读取成功，true表示成功，false表示失败
*/
bool ReadPureIMUData(ifstream& file, const DeviceType device, RAWDAT& Rawdata);

/**
* @brief       读取小推车机械编排参考结果
* @param[in]   file      ifstream&      已打开的文件流
* @param[out]  InsRef    InsEpochState& 读取一个历元的机械编排参考结果
* @return      bool  返回是否读取成功，true表示成功，false表示失败
*/
bool ReadPureINSRef(ifstream& file, InsEpochState& InsRef);
