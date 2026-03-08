/**
* @file     CoordinateTransformation.h
* @brief    坐标转换函数声明
* @details  该文件包含了坐标转换相关的函数接口，包括地心地固坐标系与大地坐标系之间的转换、\n
*           站心坐标系转换矩阵计算以及定位误差计算等功能。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/12/21
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/12/21, Weizhen Wang, Create
*/

#pragma once

#include <iostream>    // std::cerr

#include"MatrixAndVect.h"

/**
* @brief       地心地固坐标系转换为大地坐标系
* @param[in]   xyz   const double[]   地心地固坐标，xyz[0] = X, xyz[1] = Y, xyz[2] = Z
* @param[out]  BLH   double[]         大地坐标，BLH[0] = 纬度B(弧度)，BLH[1] = 经度L(弧度)，BLH[2] = 高度H(米)
* @param[in]   R     const double     参考椭球长半轴
* @param[in]   F     const double     参考椭球扁率
*/
void XYZToBLH(const double xyz[], double BLH[], const double R, const double F);

/**
* @brief       大地坐标系转换为地心地固坐标系
* @param[in]   BLH   const double[]   大地坐标，BLH[0] = 纬度B(弧度)，BLH[1] = 经度L(弧度)，BLH[2] = 高度H(米)
* @param[out]  XYZ   double[]         地心地固坐标，XYZ[0] = X，XYZ[1] = Y，XYZ[2] = Z
* @param[in]   R     const double     参考椭球长半轴
* @param[in]   F     const double     参考椭球扁率
*/
void BLHToXYZ(const double BLH[], double XYZ[], const double R, const double F);

/**
* @brief       计算站心坐标系转换矩阵
* @param[in]   BLH   const double[]   大地坐标，BLH[0] = 纬度B(弧度)，BLH[1] = 经度L(弧度)
* @param[out]  Mat   double[]         3x3旋转矩阵，用于将地心地固坐标系转换为站心坐标系
*/
void BLHToNEUMat(const double BLH[], double Mat[]);

/**
* @brief       计算定位误差（基于地心地固坐标）
* @param[in]   X0    const double[]   参考站点的精确XYZ坐标
* @param[in]   Xr    const double[]   待测站点的观测XYZ坐标
* @param[in]   R     const double     参考椭球长半轴
* @param[in]   F     const double     参考椭球扁率
* @param[out]  denu  double[]         站心坐标系下的定位误差（dE, dN, dU）
*/
void Comp_dEnu(const double X0[], const double Xr[], const double R, const double F, double denu[]);

/**
* @brief       计算定位误差（基于大地坐标）
* @param[in]   BLH0  const double[]   参考站点的精确BLH坐标，纬度和经度的单位为rad
* @param[in]   BLHr  const double[]   待测站点的观测BLH坐标，纬度和经度的单位为rad
* @param[in]   R     const double     参考椭球长半轴
* @param[in]   F     const double     参考椭球扁率
* @param[out]  denu  double[]         站心坐标系下的定位误差（dE, dN, dU）
*/
void Comp_dEnu2(const double BLH0[], const double BLHr[], const double R, const double F, double denu[]);