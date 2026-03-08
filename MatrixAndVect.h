/**
* @file		MatrixAndVect.h
* @brief    矩阵和向量基础运算接口定义
* @details  本文件为矩阵和向量运算提供标准化的函数接口声明，涵盖点积、叉积、单位化、矩阵加减乘、转置、求逆、行列删除及矩阵重组等核心运算\n
*           所有函数均对输入参数的合法性进行校验，保证运算的鲁棒性和正确性。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/01
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/01, Weizhen Wang, Create
*           2025/11/02，Weizhen Wang，添加函数声明 Normalize3(const int m, const double A[], double U[]) 用于计算三维向量的单位化向量
*           2025/12/21，Weizhen Wang，添加函数声明 SkewSymmetricMatrix(const int m, const double A[], double M[]) 用于构建反对称矩阵
*           2025/12/22，Weizhen Wang，添加函数声明 double Norm3(const int m,const double scale, const double A[]) 用于求解三维向量的模长
*           2025/12/22，Weizhen Wang，添加函数声明 EyeMatRowMajor(const int n, double I[])用于设置单位矩阵
*/

#pragma once

#include <cstdio>   // printf, fprintf
#include <cstdlib>  // malloc, free
#include <cmath>    // fabs
#include <cfloat>   // DBL_EPSILON
using namespace std;

/**
* @brief       生成一个 n×n 的单位矩阵（一维数组，按行优先存储）
* @param[in]   n        const int       矩阵的维度，应为正整数
* @param[out]  I        double[]        输出的一维数组，长度应至少为 n*n，按行优先存储单位矩阵
* @return      bool     单位矩阵生成是否成功，true 表示成功，false 表示失败
*/
bool EyeMatRowMajor(const int n, double I[]);

/**
* @brief       计算两个向量的点积（内积）
* @param[in]   m   const int      第一个向量A的维数
* @param[in]   n   const int      第二个向量B的维数
* @param[in]   A   const double[] 第一个向量数组
* @param[in]   B   const double[] 第二个向量数组
* @return      double  返回两个向量的点积，如果向量维数不合法则返回0.0
*/
double VectDot(const int m, const int n, const double A[], const double B[]);

/**
* @brief       计算两个三维向量的叉积
* @param[in]   m   const int      第一个向量A的维数
* @param[in]   n   const int      第二个向量B的维数
* @param[in]   A   const double[] 第一个三维向量数组
* @param[in]   B   const double[] 第二个三维向量数组
* @param[out]  C   double[]       存储叉积结果的三维向量数组
* @return      bool  返回是否成功，true表示成功，false表示失败
*/
bool CrossDot(const int m, const int n, const double A[], const double B[], double C[]);

/**
* @brief       反对称矩阵构建
* @param[in]   m      const int    输入向量的维度，应固定为 3。
* @param[in]   A      const double 输入三维向量，格式为 A[0], A[1], A[2]。
* @param[out]  U      double       输出3 * 3矩阵
* @return      bool   返回单位化是否成功，true 表示成功，false 表示失败。
*/

bool SkewSymmetricMatrix(const int m, const double A[], double M[]);

/**
* @brief       计算三维向量的单位向量（归一化）
* @param[in]   m   const int      输入向量的维数，应固定为3
* @param[in]   A   const double[] 输入三维向量数组，格式为A[0], A[1], A[2]
* @param[out]  U   double[]       输出的单位向量数组，U = A / |A|
* @return      bool  返回归一化是否成功，true表示成功，false表示失败
*/
bool Normalize3(const int m, const double A[], double U[]);

/**
* @brief       计算三维向量的模长（Euclidean Norm）
* @param[in]   m      const int     输入向量的维度，应固定为 3。
* @param[in]   scale  const double  向量的比力因子
* @param[in]   A      const double* 输入三维向量，格式为 A[0], A[1], A[2]。
* @return      double 返回输入向量的模长 |A|；若计算失败则返回 -1.0。
*/
double Norm3(const int m, const double scale, const double A[]);

/**
* @brief       实现两个矩阵的加法运算
* @param[in]   m   const int      矩阵行数
* @param[in]   n   const int      矩阵列数
* @param[in]   M1  const double[] 第一个矩阵（m行n列）
* @param[in]   M2  const double[] 第二个矩阵（m行n列）
* @param[out]  M3  double[]       存储加法结果的矩阵（m行n列）
* @return      bool  返回是否成功，true表示成功，false表示失败
*/
bool MatrixAddition(const int m, const int n, const double M1[], const double M2[], double M3[]);

/**
* @brief       实现两个矩阵的减法运算
* @param[in]   m   const int      矩阵行数
* @param[in]   n   const int      矩阵列数
* @param[in]   M1  const double[] 被减矩阵（m行n列）
* @param[in]   M2  const double[] 减数矩阵（m行n列）
* @param[out]  M3  double[]       存储减法结果的矩阵（m行n列）
* @return      bool  返回是否成功，true表示成功，false表示失败
*/
bool MatrixSubtraction(const int m, const int n, const double M1[], const double M2[], double M3[]);

/**
* @brief       实现两个矩阵的乘法运算
* @param[in]   m1  const int      第一个矩阵M1的行数
* @param[in]   n1  const int      第一个矩阵M1的列数，也是第二个矩阵M2的行数
* @param[in]   m2  const int      第二个矩阵M2的行数，必须等于n1
* @param[in]   n2  const int      第二个矩阵M2的列数
* @param[in]   M1  const double[] 第一个矩阵（m1行n1列）
* @param[in]   M2  const double[] 第二个矩阵（m2行n2列）
* @param[out]  M3  double[]       存储乘法结果的矩阵（m1行n2列）
* @return      bool  返回是否成功，true表示成功，false表示失败
*/
bool MatrixMultiply(const int m1, const int n1, const int m2, const int n2, const double M1[], const double M2[], double M3[]);

/**
* @brief       实现矩阵的转置运算
* @param[in]   m   const int      原矩阵M1的行数
* @param[in]   n   const int      原矩阵M1的列数
* @param[in]   M1  const double[] 原矩阵（m行n列）
* @param[out]  MT  double[]       存储转置结果的矩阵（n行m列）
* @return      bool  返回是否成功，true表示成功，false表示失败
*/
bool MatrixTranspose(const int m, const int n, const double M1[], double MT[]);

/**
* @brief       计算n阶方阵的逆矩阵
* @param[in]   n   const int      方阵的阶数
* @param[in]   a   const double[] 原矩阵（n行n列），按行优先存储
* @param[out]  b   double[]       存储逆矩阵的矩阵（n行n列），按行优先存储
* @return      bool  返回是否成功，true表示成功，false表示失败
*/
bool MatrixInv(const int n, const double a[], double b[]);

/**
* @brief       从矩阵中删除指定的一行和一列
* @param[in]   m   const int      原矩阵行数
* @param[in]   n   const int      原矩阵列数
* @param[in]   m1  int            要删除的行序号（1-based）
* @param[in]   n1  int            要删除的列序号（1-based）
* @param[in,out] M double[]       矩阵原矩阵，删除行和列后的矩阵（按行优先存储）
*/
void deleteRowAndColumn(const int m, const int n, int m1, int n1, double M[]);

/**
* @brief       删除向量中指定的一行
* @param[in]   rows        const int    原向量的行数（长度）
* @param[in]   rowToDelete const int    要删除的行序号（1-based）
* @param[in,out] vector   double[]      向量原向量，删除指定行后的向量
*/
void deleteRow(const int rows, const int rowToDelete, double vector[]);

/**
* @brief       重构矩阵的存储结构
* @param[in]   a   const int      新矩阵行数
* @param[in]   b   const int      新矩阵列数
* @param[in]   m   const int      原矩阵行数
* @param[in]   n   const int      原矩阵列数
* @param[in,out] A double[]       矩阵原矩阵（m行n列），重构后的矩阵（a行b列），按行优先存储
*/
void restructureMatrix(const int a, const int b, const int m, const int n, double A[]);