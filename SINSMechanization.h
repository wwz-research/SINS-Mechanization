/**
* @file     SINSMechanization.h
* @brief    惯导机械编排函数声明
* @details  该文件包含了惯导机械编排相关的函数接口，包括姿态更新、速度更新、位置更新等核心功能，\n
*           以及四元数运算、姿态角计算、重力计算、外推等辅助函数。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/12/24
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/12/24, Weizhen Wang, Create
*/

#pragma once

#include"baseSDC.h"
#include"MatrixAndVect.h"

/**
* @brief       四元数乘法运算
* @param[in]   p       const QUATER&   第一个四元数
* @param[in]   q       const QUATER&   第二个四元数
* @param[out]  result  QUATER&        结果四元数
*/
void QuaternionMultiply(const QUATER p, const QUATER q, QUATER& result);

/**
* @brief       根据四元数计算姿态角
* @param[in]   Q       const QUATER&   四元数
* @param[out]  atti     ATTITUDE&      姿态角（横滚、俯仰、航向，单位：rad）
*/
void CalPostureWithQuaternion(const QUATER Q, ATTITUDE& atti);

/**
* @brief       计算重力向量在导航系下的投影
* @param[in]   blh     const POSITION&  位置信息（纬度、经度、高度）
* @param[out]  gpn     double[]        重力向量在导航系下的投影（单位：m/s?）
*/
void Calgpn(const POSITION blh, double gpn[3]);

/**
* @brief       外推计算中间时刻的角速度、速度和重力
* @param[in]   prev_1   const InsEpochState&  前一个历元状态
* @param[in]   prev_2   const InsEpochState&  前两个历元状态
* @param[in]   Rawdata_cur  const RAWDAT&     当前历元原始数据
* @param[out]  wien     double[]              地球自转角速度在导航系下的投影（单位：rad/s）
* @param[out]  wenn     double[]              导航系相对地球的角速度在导航系下的投影（单位：rad/s）
* @param[out]  v        double[]              速度向量（单位：m/s）
* @param[out]  gpn      double[]              重力向量在导航系下的投影（单位：m/s?）
*/
void Extrapolation(const InsEpochState prev_1, const InsEpochState prev_2,const RAWDAT Rawdata_cur, double wien[3], double wenn[3], double v[3], double gpn[3]);

/**
* @brief       姿态更新
* @param[in]   Rawdata_cur  const RAWDAT&     当前历元原始数据
* @param[in]   Rawdata_pre  const RAWDAT&     前一个历元原始数据
* @param[in,out] InsState   InsState&        惯导状态（输入前一历元状态，输出当前历元状态）
* @return      bool  返回是否更新成功，true表示成功，false表示失败
*/
bool AttitudeUpdate(const RAWDAT Rawdata_cur, const RAWDAT Rawdata_pre, InsState& InsState);

/**
* @brief       速度更新
* @param[in]   Rawdata_cur  const RAWDAT&     当前历元原始数据
* @param[in]   Rawdata_pre  const RAWDAT&     前一个历元原始数据
* @param[in,out] InsState   InsState&        惯导状态（输入前一历元状态，输出当前历元状态）
* @return      bool  返回是否更新成功，true表示成功，false表示失败
*/
bool VelocityUpdate(const RAWDAT Rawdata_cur, const RAWDAT Rawdata_pre, InsState& InsState);

/**
* @brief       位置更新
* @param[in]   Rawdata_cur  const RAWDAT&     当前历元原始数据
* @param[in]   Rawdata_pre  const RAWDAT&     前一个历元原始数据
* @param[in,out] InsState   InsState&        惯导状态（输入前一历元状态，输出当前历元状态）
* @return      bool  返回是否更新成功，true表示成功，false表示失败
*/
bool PositionUpdate(const RAWDAT Rawdata_cur, const RAWDAT Rawdata_pre, InsState& InsState);
