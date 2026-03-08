/**
* @file     Align.cpp
* @brief    粗对准相关函数实现文件
* @details  本文件实现了粗对准相关的函数，包括姿态矩阵计算、整段数据粗对准、每秒粗对准、\n
*           每个历元粗对准以及时间噪声计算等功能。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/02
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/02, Weizhen Wang, Create
*           2025/12/26，Weizhen Wang，添加函数 CoarseAlign_Mechanization(ifstream& File, ALIGNPOS& Pos, ofstream& OutputFile)用于进行解析粗对准作为小推车惯导机械编排的初始姿态角
*/

#include"Align.h"

/**
* @param[in,out] Pos   ALIGNPOS&  粗对准位置结构体，输入比力和角速度平均值，输出姿态矩阵和姿态角
* @par History:
*              2025/11/02, Weizhen Wang, Create\n
* @internals  内部实现将比力和角速度转换为观测向量，通过叉积运算构建观测矩阵，\n
*             然后计算导航坐标系下的重力向量和地球自转角速度向量，通过归一化和矩阵乘法得到姿态矩阵。
*/
void CalCbn(ALIGNPOS& Pos)
{
	// 将平均比力、角速度转化为观测向量
	double gb[3] = { 0.0 };	// 比力
	double wb[3] = { 0.0 };	// 角速度
	double vb[3] = { 0.0 };	// 比力和角速度的叉乘
	double vgb[3] = { 0.0 };// 比力叉乘加速度叉乘比力

	for (int i = 0; i < 3; i++)
	{
		gb[i] = -Pos.billimean[i];
		wb[i] = Pos.anglevelmean[i];
	}
	CrossDot(3, 3, gb, wb, vb);	// 向量叉乘
	CrossDot(3, 3, vb, gb, vgb);

	// 定义输入向量
	double gn[3] = { 0.0, 0.0, gravity };	                // 重力输入量
	double wn[3] = { we * cos(fai), 0.0, -we * sin(fai) };	// 角速度输入量
	double vn[3] = { 0.0 }, vgn[3] = { 0.0 };
	CrossDot(3, 3, gn, wn, vn);	// 向量叉乘
	CrossDot(3, 3, vn, gn, vgn);

	// 正交化与归一化
	double wg[3] = { 0.0 }, ww[3] = { 0.0 }, wgw[3] = { 0.0 };
	Normalize3(3, gb, wg);
	Normalize3(3, vb, ww);
	Normalize3(3, vgb, wgw);

	double vg[3] = { 0.0 }, vw[3] = { 0.0 }, vgw[3] = { 0.0 };
	Normalize3(3, gn, vg);
	Normalize3(3, vn, vw);
	Normalize3(3, vgn, vgw);

	// 求解姿态矩阵
	double V[9] = { vg[0],vw[0],vgw[0],
					vg[1],vw[1],vgw[1],
					vg[2],vw[2],vgw[2] };
	double W[9] = { wg[0],wg[1] ,wg[2] ,
					ww[0],ww[1] ,ww[2] ,
					wgw[0],wgw[1] ,wgw[2] };

	MatrixMultiply(3,3,3,3,V,W,Pos.Cbn);	// 姿态矩阵
	Pos.SetPos();	// 设置姿态角
}

/**
* @param[in]   File        ifstream&   输入文件流
* @param[in,out] Pos1      ALIGNPOS&   当前历元的粗对准位置结构体
* @param[in]   Pos2        ALIGNPOS&   参考粗对准位置结构体
* @param[out]  OutputFile  ofstream&   输出文件流
* @par History:
*              2025/11/02, Weizhen Wang, Create\n
* @internals  内部实现逐历元读取IMU数据，对每个历元计算姿态角，并计算与参考姿态角的RMS误差。\n
*             每个历元使用当前历元的比力和角速度值直接计算姿态矩阵。
*/
void CoarseAlign_EveryEpoch(ifstream& File, ALIGNPOS& Pos1,const ALIGNPOS& Pos2, ofstream& OutputFile)
{
	OutputFile << "%Yaw Pitch Roll(Deg):\n";

	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;
	double rmse[3] = { 0.0,0.0,0.0 };

	// 逐行读取文件
	RAWDAT Rawdata;
	while (ReadFile_Ali(File, Align, Rawdata))
	{
		// 提取时间
		Pos1.time = Rawdata.time;

		Pos1.billimean[0] = Rawdata.acc.X;
		Pos1.billimean[1] = Rawdata.acc.Y;
		Pos1.billimean[2] = Rawdata.acc.Z;

		Pos1.anglevelmean[0] = Rawdata.gyr_v.X;	// 弧度
		Pos1.anglevelmean[1] = Rawdata.gyr_v.Y;	// 弧度
		Pos1.anglevelmean[2] = Rawdata.gyr_v.Z;	// 弧度
		epochnum++;

		// 计算姿态角
		CalCbn(Pos1);
		rmse[0] += pow(Pos1.yaw - Pos2.yaw, 2);
		rmse[1] += pow(Pos1.pitch - Pos2.pitch, 2);
		rmse[2] += pow(Pos1.roll - Pos2.roll, 2);

		OutputFile << fixed << setprecision(12);
		OutputFile << Pos1.time.Second << ",";
		OutputFile << Pos1.yaw << "," << Pos1.pitch << "," << Pos1.roll << endl;

	}
	for (int i = 0; i < 3; i++)
	{
		rmse[i] = sqrt(rmse[i] / epochnum);
	}
	printf("\n每历元计算的姿态角的rms为(yaw pitch roll)：%.6f %.6f %.6f", rmse[0], rmse[1], rmse[2]);
	cout << "\n每历元的姿态角求解成功，结果已保存在文件中！\n";

}

/**
* @param[in]   File        ifstream&   输入文件流
* @param[in,out] Pos1      ALIGNPOS&   当前秒的粗对准位置结构体
* @param[in]   Pos2        ALIGNPOS&   参考粗对准位置结构体
* @param[out]  OutputFile  ofstream&   输出文件流
* @par History:
*              2025/11/02, Weizhen Wang, Create\n
* @internals  内部实现按秒为单位累积IMU数据，计算每秒的平均比力和平均角速度，\n
*             然后使用平均值计算该秒的姿态角，并计算与参考姿态角的RMS误差。
*/
void CoarseAlign_EverySecond(ifstream& File,  ALIGNPOS& Pos1,const ALIGNPOS& Pos2, ofstream& OutputFile)
{
	OutputFile << "%Yaw Pitch Roll(Deg):\n";

	// 初始化加速度计和角速度的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0, T = 0.0;
	double totaltime = 0.0;
	double dt = 0.0;
	double rmse[3] = { 0.0, 0.0, 0.0 };

	// 逐行读取文件
	RAWDAT Rawdata;
	while (ReadFile_Ali(File, Align, Rawdata))
	{
		// 提取时间
		Pos1.time = Rawdata.time;
		if (epochnum != 0)dt = (Pos1.time.Week - Pos1.prevtime.Week) * 604800 + Pos1.time.Second - Pos1.prevtime.Second;
		totaltime += dt;
		// 当读取时间小于1秒时
		if (fabs(totaltime - 1.0) > 1e-3)
		{
			accmean[0] += Rawdata.acc.X;
			accmean[1] += Rawdata.acc.Y;
			accmean[2] += Rawdata.acc.Z;

			avelmean[0] += Rawdata.gyr_v.X;	// 弧度
			avelmean[1] += Rawdata.gyr_v.Y;	// 弧度
			avelmean[2] += Rawdata.gyr_v.Z;	// 弧度

			epochnum += 1.0;
		}
		// 等于一秒时
		else if (fabs(totaltime - 1.0) < 1e-3)
		{
			// 求平均
			for (int i = 0; i < 3; i++)
			{
				Pos1.billimean[i] = accmean[i] / epochnum;
				Pos1.anglevelmean[i] = avelmean[i] / epochnum;
			}
			// 计算姿态角
			CalCbn(Pos1);
			rmse[0] += pow(Pos1.yaw - Pos2.yaw, 2);
			rmse[1] += pow(Pos1.pitch - Pos2.pitch, 2);
			rmse[2] += pow(Pos1.roll - Pos2.roll, 2);

			T++;

			OutputFile << fixed << setprecision(0) << T << ",";
			OutputFile << fixed << setprecision(12);
			OutputFile << Pos1.yaw << "," << Pos1.pitch << "," << Pos1.roll << endl;

			// 重新初始化
			for (int i = 0; i < 3; i++)
			{
				accmean[i] = 0.0;
				avelmean[i] = 0.0;
			}
			totaltime = 0.0;
			epochnum = 1e-13;	// 避免跳过下一个历元
		}
		// 将该历元时间保存
		Pos1.prevtime = Pos1.time;
	}
	for (int i = 0; i < 3; i++)
	{
		rmse[i] = sqrt(rmse[i] / T);
	}
	printf("\n每秒平均计算的姿态角的rmse为(yaw pitch roll)：%.6f %.6f %.6f", rmse[0], rmse[1], rmse[2]);
	cout << "\n每秒平均的姿态角求解成功，结果已保存在文件中！\n";

}

/**
* @param[in]   File        ifstream&   输入文件流
* @param[out]  Pos         ALIGNPOS&   粗对准位置结构体
* @param[out]  OutputFile  ofstream&   输出文件流
* @par History:
*              2025/11/02, Weizhen Wang, Create\n
* @internals  内部实现读取整段IMU数据，计算所有数据的平均比力和平均角速度，\n
*             然后使用平均值计算整段数据的姿态角。
*/
void CoarseAlign_Whole(ifstream& File, ALIGNPOS& Pos, ofstream& OutputFile)
{
	OutputFile << "%Yaw Pitch Roll(Deg):\n";

	// 初始化加速度计和角速度的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;

	// 逐行读取文件
	RAWDAT Rawdata;
	while (ReadFile_Ali(File, Align, Rawdata))
	{
		accmean[0] += Rawdata.acc.X;
		accmean[1] += Rawdata.acc.Y;
		accmean[2] += Rawdata.acc.Z;

		avelmean[0] += Rawdata.gyr_v.X;	// 弧度
		avelmean[1] += Rawdata.gyr_v.Y;	// 弧度
		avelmean[2] += Rawdata.gyr_v.Z;	// 弧度

		epochnum += 1.0;
	}

	// 求平均
	for (int i = 0; i < 3; i++)
	{
		Pos.billimean[i] = accmean[i] / epochnum;
		Pos.anglevelmean[i] = avelmean[i] / epochnum;
	}

	// 计算姿态角
	CalCbn(Pos);

	OutputFile << fixed << setprecision(12);
	OutputFile << Pos.yaw << "," << Pos.pitch << "," << Pos.roll << endl;

	cout << "\n整段数据的平均姿态角求解成功，结果已保存在文件中！\n";
}

/**
* @param[in]   File        ifstream&   输入文件流
* @param[out]  Pos         ALIGNPOS&   粗对准位置结构体
* @param[out]  OutputFile  ofstream&   输出文件流
* @par History:
*              2025/11/26, Weizhen Wang, Create\n
* @internals  内部实现读取小推车的整段静态IMU数据，计算所有数据的平均比力和平均角速度，\n
*             然后使用平均值计算整段数据的姿态角作为初始姿态角。
*/
void CoarseAlign_Mechanization(ifstream& File, ALIGNPOS& Pos, ofstream& OutputFile)
{
	OutputFile << "%Yaw Pitch Roll(Deg):\n";

	// 初始化加速度计和角速度的平均值
	double accmean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double avelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;

	// 逐行读取文件
	RAWDAT Rawdata;
	while (ReadPureIMUData(File, Mechanization, Rawdata))
	{
		accmean[0] += Rawdata.acc.X;
		accmean[1] += Rawdata.acc.Y;
		accmean[2] += Rawdata.acc.Z;

		avelmean[0] += Rawdata.gyr_v.X;	// 弧度
		avelmean[1] += Rawdata.gyr_v.Y;	// 弧度
		avelmean[2] += Rawdata.gyr_v.Z;	// 弧度

		epochnum += 1.0;
	}

	// 求平均
	for (int i = 0; i < 3; i++)
	{
		Pos.billimean[i] = accmean[i] / epochnum;
		Pos.anglevelmean[i] = avelmean[i] / epochnum;
	}

	// 计算姿态角
	CalCbn(Pos);

	OutputFile << fixed << setprecision(12);
	OutputFile << Pos.yaw << "," << Pos.pitch << "," << Pos.roll << endl;

	cout << "\n整段数据的平均姿态角求解成功，结果已保存在文件中！\n";
}

/**
* @param[in]   File        ifstream&   输入文件流
* @param[out]  OutputFile  ofstream&   输出文件流
* @par History:
*              2025/11/02, Weizhen Wang, Create\n
* @internals  内部实现计算白噪声随时间变化的规律，输出零偏和随时间变化的白噪声值。\n
*             白噪声计算公式考虑了地球自转角速度、转台纬度和时间因子。
*/
void CalTimeNoise(ifstream& File, ofstream& OutputFile)
{
	OutputFile << "%Bias(deg): " << fixed << setprecision(12) << Deg * ((0.05 / 3600.0) / (we * cos(fai))) << endl;
	OutputFile << "%Time(s),WhiteNoise(deg):\n";

	// 初始化加速度计和角速度的平均值
	double t0 = 0.0, totalt = 0;
	double epochnum = 0.0;
	// 逐行读取文件
	RAWDAT Rawdata;
	while (ReadFile_Ali(File, Align, Rawdata))
	{
		if (epochnum == 0) { t0 = Rawdata.time.Week * 604800 + Rawdata.time.Second; }	// 取初始时间
		epochnum++;
		totalt = Rawdata.time.Week * 604800 + Rawdata.time.Second - t0;
		if (epochnum == 1)continue;	// 避免totalt为0

		OutputFile << fixed << setprecision(4);
		OutputFile << totalt << ",";
		OutputFile << fixed << setprecision(12);
		OutputFile << Deg * ((0.005 / 3600.0) / ((we * cos(fai)) * sqrt(totalt))) << endl;
	}
	cout << "\n白噪声误差随时间变化的结果计算成功，结果已保存在文件中！\n";
}