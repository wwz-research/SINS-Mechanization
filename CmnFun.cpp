/**
* @file     CmnFun.cpp
* @brief    惯导机械编排辅助函数实现
* @details  该文件包含了惯导机械编排中需要使用到的辅助函数，包括四元数运算、姿态角计算、\n
*           重力计算和外推等功能的实现。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/12/24
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/12/24, Weizhen Wang, Create
*/

#include"SINSMechanization.h"

/**
* @param[in]   p       const QUATER&   第一个四元数
* @param[in]   q       const QUATER&   第二个四元数
* @param[out]  result  QUATER&        结果四元数
* @return      无
* @note        计算两个四元数的乘积，result = p ? q。\n
*              四元数乘法用于姿态更新的旋转组合。
* @par History:
*              2025/12/24, Weizhen Wang, Create
* @internals   四元数乘法公式：\n
*              result.q[0] = p.q[0]*q.q[0] - p.q[1]*q.q[1] - p.q[2]*q.q[2] - p.q[3]*q.q[3]\n
*              result.q[1] = p.q[0]*q.q[1] + p.q[1]*q.q[0] + p.q[2]*q.q[3] - p.q[3]*q.q[2]\n
*              result.q[2] = p.q[0]*q.q[2] - p.q[1]*q.q[3] + p.q[2]*q.q[0] + p.q[3]*q.q[1]\n
*              result.q[3] = p.q[0]*q.q[3] + p.q[1]*q.q[2] - p.q[2]*q.q[1] + p.q[3]*q.q[0]
*/
void QuaternionMultiply(const QUATER p, const QUATER q, QUATER& result)
{
    result.q[0] = p.q[0] * q.q[0] - p.q[1] * q.q[1] - p.q[2] * q.q[2] - p.q[3] * q.q[3];
    result.q[1] = p.q[0] * q.q[1] + p.q[1] * q.q[0] + p.q[2] * q.q[3] - p.q[3] * q.q[2];
    result.q[2] = p.q[0] * q.q[2] - p.q[1] * q.q[3] + p.q[2] * q.q[0] + p.q[3] * q.q[1];
    result.q[3] = p.q[0] * q.q[3] + p.q[1] * q.q[2] - p.q[2] * q.q[1] + p.q[3] * q.q[0];
}

/**
* @param[in]   Q       const QUATER&   四元数
* @param[out]  atti     ATTITUDE&      姿态角（横滚、俯仰、航向，单位：rad）
* @return      无
* @note        根据四元数计算姿态角（横滚角、俯仰角、航向角）。\n
*              通过方向余弦矩阵的元素计算姿态角。
* @par History:
*              2025/12/24, Weizhen Wang, Create
* @internals   计算步骤：\n
*              1. 根据四元数计算方向余弦矩阵的元素C31, C32, C33\n
*              2. 横滚角 roll = atan2(C32, C33)\n
*              3. 俯仰角 pitch = atan(-C31 / sqrt(C32? + C33?))\n
*              4. 根据四元数计算方向余弦矩阵的元素C11, C21\n
*              5. 航向角 yaw = atan2(C21, C11)
*/
void CalPostureWithQuaternion(const QUATER Q, ATTITUDE& atti)
{
    double C32 = 2.0 * (Q.q[2] * Q.q[3] + Q.q[0] * Q.q[1]);
    double C33 = pow(Q.q[0], 2) - pow(Q.q[1], 2) - pow(Q.q[2], 2) + pow(Q.q[3], 2);
    atti.roll = atan2(C32, C33);                           // 横滚角(rad)

    double C31 = 2.0 * (Q.q[1] * Q.q[3] - Q.q[0] * Q.q[2]);
    atti.pitch = atan(-C31 / sqrt(C32 * C32 + C33 * C33));  // 俯仰角(rad)

    double C11 = pow(Q.q[0], 2) + pow(Q.q[1], 2) - pow(Q.q[2], 2) - pow(Q.q[3], 2);
    double C21 = 2.0 * (Q.q[1] * Q.q[2] + Q.q[0] * Q.q[3]);
    atti.yaw = atan2(C21, C11);                              // 航向角(rad)
}

/**
* @param[in]   blh     const POSITION&  位置信息（纬度、经度、高度）
* @param[out]  gpn     double[]        重力向量在导航系下的投影（单位：m/s?）
* @return      无
* @note        计算重力向量在导航系下的投影。\n
*              使用WGS84椭球模型，考虑纬度和高度对重力的影响。
* @par History:
*              2025/12/24, Weizhen Wang, Create
* @internals   计算步骤：\n
*              1. 计算基准重力加速度g0，考虑纬度影响\n
*              2. 计算考虑高度影响的重力加速度g_faih\n
*              3. 在导航系下，重力向量为[0, 0, g_faih]（北、东、天）
*/
void Calgpn(const POSITION blh, double gpn[3])
{
    double g0;
    g0 = 9.7803267715 * (1 + 0.0052790414 * pow(sin(blh.latitude), 2) + 0.0000232718 * pow(sin(blh.latitude), 4));

    double g_faih;
    g_faih = g0 - (3.087691089e-6 - 4.397731e-9 * pow(sin(blh.latitude), 2)) * blh.H + 0.721e-12 * pow(blh.H, 2);

    gpn[0] = 0.0;
    gpn[1] = 0.0;
    gpn[2] = g_faih;

}


/**
* @param[in]   prev_1      const InsEpochState&  前一个历元状态
* @param[in]   prev_2      const InsEpochState&  前两个历元状态
* @param[in]   Rawdata_cur const RAWDAT&         当前历元原始数据
* @param[out]  wien        double[]               地球自转角速度在导航系下的投影（单位：rad/s）
* @param[out]  wenn        double[]               导航系相对地球的角速度在导航系下的投影（单位：rad/s）
* @param[out]  v           double[]               速度向量（单位：m/s）
* @param[out]  gpn         double[]               重力向量在导航系下的投影（单位：m/s?）
* @return      无
* @note        外推计算中间时刻（k-1/2时刻）的角速度、速度和重力。\n
*              使用线性外推方法，基于前两个历元的状态进行预测。
* @par History:
*              2025/12/24, Weizhen Wang, Create
* @internals   计算步骤：\n
*              1. 计算prev_1和prev_2时刻的地球自转角速度wien和导航系角速度wenn\n
*              2. 计算时间间隔dt1（prev_1和prev_2之间）和dt2（当前时刻到prev_1的一半）\n
*              3. 使用线性外推计算中间时刻的wien和wenn\n
*              4. 使用线性外推计算中间时刻的速度v\n
*              5. 计算prev_1和prev_2时刻的重力，然后外推到中间时刻
*/
void Extrapolation(const InsEpochState prev_1, const InsEpochState prev_2,const RAWDAT Rawdata_cur, double wien[3], double wenn[3], double v[3],double gpn[3])
{
    // 外推角速度
    double R_M_prev1 = R_WGS84 * (1 - E2_WGS84) / sqrt(pow(1 - E2_WGS84 * sin(prev_1.pos.latitude) * sin(prev_1.pos.latitude), 3));
    double R_N_prev1 = R_WGS84 / sqrt(1 - E2_WGS84 * sin(prev_1.pos.latitude) * sin(prev_1.pos.latitude));
    double R_M_prev2 = R_WGS84 * (1 - E2_WGS84) / sqrt(pow(1 - E2_WGS84 * sin(prev_2.pos.latitude) * sin(prev_2.pos.latitude), 3));
    double R_N_prev2 = R_WGS84 / sqrt(1 - E2_WGS84 * sin(prev_2.pos.latitude) * sin(prev_2.pos.latitude));

    double wenn_prev1[3] = { prev_1.vel.Ve / (R_N_prev1 + prev_1.pos.H),-prev_1.vel.Vn / (R_M_prev1 + prev_1.pos.H),-prev_1.vel.Ve * tan(prev_1.pos.latitude) / (R_N_prev1 + prev_1.pos.H) };
    double wien_prev1[3] = { we * cos(prev_1.pos.latitude),0.0,-we * sin(prev_1.pos.latitude) };
    double wenn_prev2[3] = { prev_2.vel.Ve / (R_N_prev2 + prev_2.pos.H),-prev_2.vel.Vn / (R_M_prev2 + prev_2.pos.H),-prev_2.vel.Ve * tan(prev_2.pos.latitude) / (R_N_prev2 + prev_2.pos.H) };
    double wien_prev2[3] = { we * cos(prev_2.pos.latitude),0.0,-we * sin(prev_2.pos.latitude) };

    double dt1 = (prev_1.time.Week - prev_2.time.Week) * 604800.0 + (prev_1.time.Second - prev_2.time.Second);
    double dt2 = ((Rawdata_cur.time.Week - prev_1.time.Week) * 604800.0 + (Rawdata_cur.time.Second - prev_1.time.Second))/2.0;

    for (int i = 0; i < 3; i++)
    {
        wien[i] = wien_prev1[i] + dt2 * (wien_prev1[i] - wien_prev2[i]) / dt1;
        wenn[i] = wenn_prev1[i] + dt2 * (wenn_prev1[i] - wenn_prev2[i]) / dt1;
    }

    // 外推速度差值
    double v_prev1[3] = { prev_1.vel.Vn, prev_1.vel.Ve, prev_1.vel.Vd };
    double v_prev2[3] = { prev_2.vel.Vn, prev_2.vel.Ve, prev_2.vel.Vd };

    for (int j = 0; j < 3; j++)
    {
        v[j] = v_prev1[j] + dt2 * (v_prev1[j] - v_prev2[j]) / dt1;
    }

    // 外推重力
    double gpn_prev1[3] = { 0.0 } , gpn_prev2[3] = {0.0};
    Calgpn(prev_1.pos, gpn_prev1);
    Calgpn(prev_2.pos, gpn_prev2);

    for (int k = 0; k < 3; k++)
    {
        gpn[k] = gpn_prev1[k] + dt2 * (gpn_prev1[k] - gpn_prev2[k]) / dt1;
    }
}