/**
* @file     Calibration.cpp
* @brief    标定相关函数实现文件
* @details  本文件实现了加速度计和陀螺仪标定相关的函数，包括六位置法标定加速度计、\n
*           角位置法标定陀螺仪以及误差补偿等功能。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/01
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/01, Weizhen Wang, Create
*/

#include"Calibration.h"

/**
* @param[in]   File    ifstream&        输入文件流
* @param[in]   Place   const int        标定位置，x_up x_down y_up y_down z_up z_down 分别为 0 1 2 3 4 5
* @param[out]  Error   AccCaliError&    加速度计标定误差结构体
* @par History:
*             2025/11/01, Weizhen Wang, Create\n
* @internals  内部实现读取标定位置文件，累加所有历元的加速度计数据，计算该位置的平均比力值，\n
*             并将结果存储到Error.bilimean数组中对应的位置。
*/
void CalCaliBillMean(ifstream& File, const int Place,  AccCaliError& Error)
{
	// 初始化加速度计三个方向的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double epochnum = 0.0;

	// 逐行读取文件
	RAWDAT Rawdata;
	while (ReadFile_Cali(File, Calibraton, Rawdata))
	{
		accmean[0] += Rawdata.acc.X;
		accmean[1] += Rawdata.acc.Y;
		accmean[2] += Rawdata.acc.Z;
		epochnum += 1.0;
	}
	for (int i = 0; i < 3; i++) { Error.bilimean[i * 6 + Place] = accmean[i] / epochnum; }   
}

/**
* @param[in,out] Error      AccCaliError&   加速度计标定误差结构体，输入比力平均值，输出标定矩阵
* @param[out]    OutputFile  ofstream&      输出文件流
* @par History:
*              2025/11/01, Weizhen Wang, Create\n
* @internals  内部实现使用六位置法计算加速度计标定矩阵，通过最小二乘法求解M=L*A_T*(A*A_T)^-1，\n
*             其中L为实测比力平均值，A为理论比力值矩阵。然后提取零偏、标度因子和交叉耦合误差，并输出到文件。
*/
void CalAcc(AccCaliError& Error, ofstream& OutputFile)
{
	// 填充真实比力值4*6
	double bilireal[24] = { 0.0 };
	bilireal[0] = bilireal[8] = bilireal[16] = gravity;
	bilireal[1] = bilireal[9] = bilireal[17] = -gravity;
	bilireal[18] = bilireal[19] = bilireal[20] = bilireal[21] = bilireal[22] = bilireal[23] = 1.0;

	// 待估参数M=L*A_T*(A*A_T)_-1,L=Error.bilimean,A=bilireal,M=Error.calibration
	double A_T[24] = { 0.0 };   // A_T
	MatrixTranspose(4, 6, bilireal, A_T);
	double M1[12] = { 0.0 };    // =L * A_T
	MatrixMultiply(3, 6, 6, 4, Error.bilimean, A_T, M1);
	double M2[16] = { 0.0 };    // =A * A_T
	MatrixMultiply(4, 6, 6, 4, bilireal, A_T, M2);
	double M3[16] = { 0.0 };    // =(A * A_T)_-1
	MatrixInv(4, M2, M3);
	MatrixMultiply(3, 4, 4, 4, M1, M3, Error.calibration);

	// 提取矩阵中的元素：零偏、比例因子、交轴耦合误差
	Error.SetBias();
	Error.SetScale();
	Error.SetCross();

	// 将结果保存在文件中
	OutputFile << fixed << setprecision(12);
	OutputFile << "%AccBias_XYZ(m/s^2):\n"
		<< Error.bias[0] << "," << Error.bias[1] << "," << Error.bias[2] << endl;

	OutputFile << "%AccScale_XYZ:\n"
		<< Error.scale[0] << "," << Error.scale[1] << "," << Error.scale[2] << endl;

	OutputFile << "%AccCross_XYZ:\n"
		<< 0.0 << "," << Error.cross[0] << "," << Error.cross[1] << endl
		<< Error.cross[2] << "," << 0.0 << "," << Error.cross[3] << endl
		<< Error.cross[4] << "," << Error.cross[5] << "," << 0.0 << endl;

	cout << "\n加速度计标定成功，结果已经保存在文件中！\n";
}

/**
* @param[in]   File        ifstream&           输入文件流
* @param[in]   Error       const AccCaliError& 加速度计标定误差结构体
* @param[out]  OutputFile  ofstream&           输出文件流
* @par History:
*              2025/11/01, Weizhen Wang, Create\n
* @internals  内部实现根据标定矩阵对原始加速度计数据进行补偿，通过公式Com = A^-1*(Raw-b)计算补偿后的值，\n
*             其中A为标定矩阵的3x3部分，b为零偏向量。输出补偿前后的加速度值。
*/
void CompenAcc(ifstream& File, const AccCaliError& Error, ofstream& OutputFile)
{
	OutputFile << "%Time Compensation Raw(m/s^2):\n";

	// 逐行读取文件
	RAWDAT Rawdata;
	while (ReadFile_Cali(File, Calibraton, Rawdata))
	{
		double Com[3] = { 0.0 };     // 补偿后
		double Raw[3] = { Rawdata.acc.X, Rawdata.acc.Y, Rawdata.acc.Z };     // 原始数据

		//Raw=A*Com+b
		double A[9] = { Error.calibration[0], Error.calibration[1], Error.calibration[2],
						Error.calibration[4], Error.calibration[5], Error.calibration[6],
						Error.calibration[8], Error.calibration[9], Error.calibration[10] };
		double b[3] = { Error.calibration[3], Error.calibration[7], Error.calibration[11] };
		double C1[3] = { 0.0 };            // =Raw-b
		MatrixSubtraction(3, 1, Raw, b, C1);
		double C2[9] = { 0.0 };            // =A_-1
		MatrixInv(3, A, C2);
		MatrixMultiply(3, 3, 3, 1, C2, C1, Com);

		OutputFile << fixed << setprecision(2);
		//OutputFile << Rawdata.time.Week << ",";
		OutputFile << Rawdata.time.Second << ",";

		OutputFile << fixed << setprecision(12);
		OutputFile << Com[0] << "," << Com[1] << "," << Com[2] << ","
			<< Raw[0] << "," << Raw[1] << "," << Raw[2] << endl;
	}
	cout << "\n已对加速度进行补偿，原始与补偿后的加速度数据已保存在文件！\n";
}

/**
* @param[in]   File_up     ifstream&           轴朝上的输入文件流
* @param[in]   File_down   ifstream&           轴朝下的输入文件流
* @param[in]   place       const int           标定轴，x、y、z分别为0、1、2
* @param[out]  Error       GyroCaliError&      陀螺仪标定误差结构体
* @par History:
*              2025/11/01, Weizhen Wang, Create\n
* @internals  内部实现利用加速度计六位置法中采集的静态陀螺数据标定陀螺零偏，\n
*             分别计算轴朝上和轴朝下时的平均角速度，零偏为两者平均值，并转换为度/小时单位。
*/
void CalGyrBias(ifstream& File_up, ifstream& File_down, const int place, GyroCaliError& Error)
{
	// 初始化陀螺仪三个方向的平均值
	double w_up = 0.0, w_down = 0.0;
	double gyrmean_up[3] = { 0.0}, gyrmean_down[3] = { 0.0 };
	double epochnum_up = 0.0, epochnum_down = 0.0;

	RAWDAT Rawdata;
	while (ReadFile_Cali(File_up, Calibraton, Rawdata))
	{
		gyrmean_up[0] += Rawdata.gyr_v.X;
		gyrmean_up[1] += Rawdata.gyr_v.Y;
		gyrmean_up[2] += Rawdata.gyr_v.Z;

		epochnum_up += 1.0;
	}
	w_up = gyrmean_up[place] / epochnum_up;

	while (ReadFile_Cali(File_down, Calibraton, Rawdata))
	{
		gyrmean_down[0] += Rawdata.gyr_v.X;
		gyrmean_down[1] += Rawdata.gyr_v.Y;
		gyrmean_down[2] += Rawdata.gyr_v.Z;

		epochnum_down += 1.0;
	}
	w_down = gyrmean_down[place] / epochnum_down;

	// 计算陀螺零偏 rad/s->deg/h
	Error.bias[place] = (w_up + w_down) / 2 * Deg * 3600.0;
}

/**
* @param[in]   File_pos    ifstream&           正向旋转的输入文件流
* @param[in]   File_neg    ifstream&           反向旋转的输入文件流
* @param[in]   place       const int           标定轴，x、y、z分别为0、1、2
* @param[out]  Error       GyroCaliError&      陀螺仪标定误差结构体
* @par History:
*              2025/11/01, Weizhen Wang, Create\n
* @internals  内部实现对角速度进行积分对标定数据标度因子，分别累加正向和反向旋转时的角增量，\n
*             标度因子误差计算公式为((gyr_pos - gyr_neg) / 2 / 2 / PAI - 1) * Deg。\n
*             注意：读取数据时间段应该是该轴转台旋转的时间段，正反向旋转数据长度应严格相等。
*/
void CalGyrScale(ifstream& File_pos, ifstream& File_neg, const int place, GyroCaliError& Error)
{
	double gyr_pos[3] = { 0.0 }, gyr_neg[3] = { 0.0 };

	RAWDAT Rawdata;
	while (ReadFile_Cali(File_pos, Calibraton, Rawdata))
	{
		gyr_pos[0] += Rawdata.gyr.X;
		gyr_pos[1] += Rawdata.gyr.Y;
		gyr_pos[2] += Rawdata.gyr.Z;
	}
	while (ReadFile_Cali(File_neg, Calibraton, Rawdata))
	{
		gyr_neg[0] += Rawdata.gyr.X;
		gyr_neg[1] += Rawdata.gyr.Y;
		gyr_neg[2] += Rawdata.gyr.Z;
	}

	// 计算陀螺比例因子
	Error.scale[place] = ((gyr_pos[place] - gyr_neg[place]) / 2 / 2 / PAI - 1) * Deg;
}

/**
* @param[in]   File        ifstream&               输入文件流
* @param[in]   place       const int               标定轴，x、y、z分别为0、1、2
* @param[in]   Error       const GyroCaliError&    陀螺仪标定误差结构体
* @param[out]  OutputFile  ofstream&               输出文件流
* @par History:
*              2025/11/01, Weizhen Wang, Create\n
* @internals  内部实现根据标定参数对原始陀螺仪数据进行补偿，通过公式Com = A^-1*(Raw-b)计算补偿后的值，\n
*             其中A为对角矩阵（包含标度因子），b为零偏向量。输出指定轴的补偿前后角速度值。
*/
void CompenGyro(ifstream& File,const int place, const GyroCaliError& Error, ofstream& OutputFile)
{
	OutputFile << "%Time Compensation_XYZ Raw_XYZ(Rad/s):\n";

	// 逐行读取文件
	RAWDAT Rawdata;
	while (ReadFile_Cali(File, Calibraton, Rawdata))
	{
		double Com[3] = { 0.0 };     // 补偿后
		double Raw[3] = { Rawdata.gyr_v.X, Rawdata.gyr_v.Y, Rawdata.gyr_v.Z };     // 原始数据 rad/s

		//Raw=A*Com+b
		double A[9] = { Error.scale[0] * Rad +1.0, 0.0, 0.0,
						0.0, Error.scale[1] * Rad +1.0, 0.0,
						0.0, 0.0, Error.scale[2] * Rad +1.0 };
		double b[3] = { Error.bias[0] * Rad / 3600.0, Error.bias[1] * Rad / 3600.0, Error.bias[2] * Rad / 3600.0 };
		double C1[3] = { 0.0 };            // =Raw-b
		MatrixSubtraction(3, 1, Raw, b, C1);
		double C2[9] = { 0.0 };            // =A_-1
		MatrixInv(3, A, C2);
		MatrixMultiply(3, 3, 3, 1, C2, C1, Com);

		OutputFile << fixed << setprecision(2);
		//OutputFile << Rawdata.time.Week << ",";
		OutputFile << Rawdata.time.Second << ",";

		OutputFile << fixed << setprecision(12);
		switch (place)
		{
		case 0:
			//OutputFile << Com[0] - we * sin(fai) * Rad << "," << Raw[0] << endl;           
			OutputFile << Com[0] << "," << Raw[0] << endl;
			break;
		case 1:
			//OutputFile << Com[1] - we * sin(fai) * Rad << "," << Raw[1] << endl;
			OutputFile << Com[1] << "," << Raw[1] << endl;
			break;
		case 2:
			//OutputFile << Com[2] - we * sin(fai) * Rad << "," << Raw[2] << endl;
			OutputFile << Com[2]  << "," << Raw[2] << endl;
			break;
		case 3:
			OutputFile << Com[0] << "," << Com[1] << "," << Com[2] << endl;
		}
	}
	cout << "\n已对角速度进行补偿，原始与补偿后的角速度数据已保存在文件！\n";
}