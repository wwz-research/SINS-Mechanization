#include"CoordinateTransformation.h"

/**
* @param[in]   xyz   const double[]   地心地固坐标，xyz[0] = X, xyz[1] = Y, xyz[2] = Z
* @param[out]  BLH   double[]         大地坐标，BLH[0] = 纬度B(弧度)，BLH[1] = 经度L(弧度)，BLH[2] = 高度H(米)
* @param[in]   R     const double     参考椭球长半轴
* @param[in]   F     const double     参考椭球扁率
* @return      无
* @note        将地心地固坐标系转换为大地坐标系。\n
*              采用迭代方法计算纬度，直到满足精度要求或达到最大迭代次数。
* @par History:
*              2025/12/21, Weizhen Wang, Create
* @internals   使用迭代方法计算纬度B，通过不断更新deltaZ直到收敛。\n
*              计算经度L使用atan2函数，计算高度H需要先计算卯酉圈半径N。
*/
void XYZToBLH(const double xyz[], double BLH[], const double R, const double F) {
    // 计算偏心率
    double e2 = 2 * F - F * F;

    // 初始化ΔZ
    double deltaZ = e2 * xyz[2];

    // 迭代参数
    const int maxIterations = 100; // 最大迭代次数
    const double epsilon = 1e-12;  // 精度阈值
    int iterationCount = 0;        // 迭代计数器

    // 迭代计算ΔZ，直到满足精度要求或达到最大迭代次数
    double prevDeltaZ = 0;
    //bool converged = false;
    while (std::abs(deltaZ - prevDeltaZ) > epsilon && iterationCount < maxIterations) {
        prevDeltaZ = deltaZ;

        // 计算纬度B
        double sqrtXY = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
        double B = std::atan((xyz[2] + deltaZ) / sqrtXY);

        // 计算sin(B)
        double sinB = std::sin(B);

        // 计算N
        double N = R / std::sqrt(1 - e2 * sinB * sinB);

        // 更新ΔZ
        deltaZ = N * e2 * sinB;

        iterationCount++;
    }

    if (iterationCount >= maxIterations) {
        std::cerr << "Warning: Iteration did not converge within " << maxIterations << " iterations." << std::endl;
    }

    // 计算经度L
    BLH[1] = std::atan2(xyz[1], xyz[0]);

    // 计算纬度B
    double sqrtXY = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
    BLH[0] = std::atan((xyz[2] + deltaZ) / sqrtXY);

    // 计算高度H
    double N = R / std::sqrt(1 - e2 * std::sin(BLH[0]) * std::sin(BLH[0]));
    BLH[2] = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + (xyz[2] + deltaZ) * (xyz[2] + deltaZ)) - N;

}

/**
* @param[in]   BLH   const double[]   大地坐标，BLH[0] = 纬度B(弧度)，BLH[1] = 经度L(弧度)，BLH[2] = 高度H(米)
* @param[out]  XYZ   double[]         地心地固坐标，XYZ[0] = X，XYZ[1] = Y，XYZ[2] = Z
* @param[in]   R     const double     参考椭球长半轴
* @param[in]   F     const double     参考椭球扁率
* @return      无
* @note        将大地坐标系转换为地心地固坐标系。\n
*              首先计算卯酉圈半径N，然后根据公式计算X、Y、Z坐标。
* @par History:
*              2025/12/21, Weizhen Wang, Create
* @internals   计算偏心率平方e2 = 2*F - F*F。\n
*              计算卯酉圈半径N = R / sqrt(1 - e2*sin^2(B))。\n
*              根据公式计算XYZ坐标：X = (N+H)*cos(B)*cos(L), Y = (N+H)*cos(B)*sin(L), Z = (N*(1-e2)+H)*sin(B)。
*/
void BLHToXYZ(const double BLH[], double XYZ[], const double R, const double F) {


    // 计算偏心率
    double e2 = 2 * F - F * F;

    // 计算卯酉圈半径N
    double N = R / std::sqrt(1 - e2 * sin(BLH[0]) * sin(BLH[0]));

    // 计算X、Y、Z坐标
    XYZ[0] = (N + BLH[2]) * cos(BLH[0]) * cos(BLH[1]);
    XYZ[1] = (N + BLH[2]) * cos(BLH[0]) * sin(BLH[1]);
    XYZ[2] = (N * (1 - e2) + BLH[2]) * sin(BLH[0]);
}

/**
* @param[in]   BLH   const double[]   大地坐标，BLH[0] = 纬度B(弧度)，BLH[1] = 经度L(弧度)
* @param[out]  Mat   double[]         3x3旋转矩阵，用于将地心地固坐标系转换为站心坐标系
* @return      无
* @note        计算站心坐标系（东北天坐标系）的旋转矩阵。\n
*              矩阵按行优先存储，用于将地心地固坐标系下的向量转换到站心坐标系。
* @par History:
*              2025/12/21, Weizhen Wang, Create
* @internals   旋转矩阵的构造：\n
*              第一行（东向）：[-sin(L), cos(L), 0]\n
*              第二行（北向）：[-sin(B)*cos(L), -sin(B)*sin(L), cos(B)]\n
*              第三行（天向）：[cos(B)*cos(L), cos(B)*sin(L), sin(B)]
*/
void BLHToNEUMat(const double BLH[], double Mat[]) {

    // 构建旋转矩阵
    Mat[0] = -sin(BLH[1]);
    Mat[1] = cos(BLH[1]);
    Mat[2] = 0.0;

    Mat[3] = -sin(BLH[0]) * cos(BLH[1]);
    Mat[4] = -sin(BLH[0]) * sin(BLH[1]);
    Mat[5] = cos(BLH[0]);

    Mat[6] = cos(BLH[0]) * cos(BLH[1]);
    Mat[7] = cos(BLH[0]) * sin(BLH[1]);
    Mat[8] = sin(BLH[0]);
}


/*
* @param[in]   X0    const double[]   参考站点的精确XYZ坐标
* @param[in]   Xr    const double[]   待测站点的观测XYZ坐标
* @param[in]   R     const double     参考椭球长半轴
* @param[in]   F     const double     参考椭球扁率
* @param[out]  denu  double[]         站心坐标系下的定位误差（dE, dN, dU）
* @return      无
* @note        计算定位误差，将地心地固坐标系下的坐标差转换到站心坐标系。\n
*              首先将待测站点的XYZ坐标转换为BLH，然后计算站心坐标系旋转矩阵，最后将坐标差转换到站心坐标系。
* @par History:
*              2025/12/21, Weizhen Wang, Create
* @internals   计算步骤：\n
*              1. 将待测站点的XYZ坐标转换为BLH坐标\n
*              2. 根据BLH坐标计算站心坐标系旋转矩阵\n
*              3. 计算坐标差dx = Xr - X0\n
*              4. 通过矩阵乘法将坐标差转换到站心坐标系：denu = Mat * dx
*/
void Comp_dEnu(const double X0[], const double Xr[], const double R, const double F, double denu[]) {
    // 创建 C 风格数组来存储旋转矩阵
    double Mat[3 * 3];
    double BLH[3];
    XYZToBLH(Xr, BLH, R, F);
    BLHToNEUMat(BLH, Mat);

    // 计算坐标差
    double dx[3] = { Xr[0] - X0[0], Xr[1] - X0[1], Xr[2] - X0[2] };

    // 将坐标差转换到地平坐标系
    MatrixMultiply(3, 3, 3, 1, Mat, dx, denu);
}

/*
* @param[in]   BLH0  const double[]   参考站点的精确BLH坐标，纬度和经度的单位为rad
* @param[in]   BLHr  const double[]   待测站点的观测BLH坐标，纬度和经度的单位为rad
* @param[in]   R     const double     参考椭球长半轴
* @param[in]   F     const double     参考椭球扁率
* @param[out]  denu  double[]         站心坐标系下的定位误差（dE, dN, dU）
* @return      无
* @note        计算定位误差，将大地坐标系下的坐标差转换到站心坐标系。\n
*              首先将参考站点和待测站点的BLH坐标转换为XYZ坐标，然后计算站心坐标系旋转矩阵，最后将坐标差转换到站心坐标系。
* @par History:
*              2025/12/21, Weizhen Wang, Create
* @internals   计算步骤：\n
*              1. 根据待测站点的BLH坐标计算站心坐标系旋转矩阵\n
*              2. 将参考站点和待测站点的BLH坐标转换为XYZ坐标\n
*              3. 计算坐标差dx = Xr - X0\n
*              4. 通过矩阵乘法将坐标差转换到站心坐标系：denu = Mat * dx
*/
void Comp_dEnu2(const double BLH0[], const double BLHr[], const double R, const double F, double denu[]) {
    // 创建 C 风格数组来存储旋转矩阵
    double Mat[3 * 3];
    BLHToNEUMat(BLHr, Mat);

    // 计算坐标差
    double X0[3] = { 0.0 }, Xr[3] = { 0.0 };
    BLHToXYZ(BLH0, X0, R, F);
    BLHToXYZ(BLHr, Xr, R, F);
    double dx[3] = { Xr[0] - X0[0], Xr[1] - X0[1], Xr[2] - X0[2] };

    // 将坐标差转换到地平坐标系
    MatrixMultiply(3, 3, 3, 1, Mat, dx, denu);
}