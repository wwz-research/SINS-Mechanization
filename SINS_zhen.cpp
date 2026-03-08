/**
* @file     SINS_zhen.cpp
* @brief    SINS系统主程序
* @details  该文件是SINS系统的主程序入口，提供了加速度计标定、陀螺仪标定、粗对准、\n
*           示例数据机械编排和小推车数据机械编排等功能模块。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/02
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/02, Weizhen Wang, Create
*           2025/12/21, Weizhen Wang, 添加mode 3功能，支持示例数据机械编排
*           2025/12/24, Weizhen Wang, 添加mode 4功能，支持小推车数据机械编排和零速修正
*/
#include <iostream>

#include"ReadFile.h"
#include"Calibration.h"
#include"Align.h"
#include"MatrixAndVect.h"
#include"CoordinateTransformation.h"
#include"SINSMechanization.h"

/**
* @return      int  返回程序执行状态，0表示正常结束
* @note        主函数，提供交互式菜单选择不同的功能模块：\n
*              mode 0: 加速度计标定 - 使用六位置法标定加速度计，计算误差矩阵并补偿\n
*              mode 1: 陀螺仪标定 - 使用角位置法标定陀螺仪，计算零偏和比力因子并补偿\n
*              mode 2: 粗对准 - 计算整段数据、每秒和每个历元的姿态角，分析白噪声\n
*              mode 3: 示例数据机械编排 - 使用示例数据进行惯导机械编排，与参考值比较\n
*              mode 4: 小推车数据机械编排 - 使用小推车实测数据进行机械编排，支持零速修正
* @par History:
*              2025/11/02, Weizhen Wang, Create
*              2025/12/21, Weizhen Wang, 添加mode 3功能，支持示例数据机械编排
*              2025/12/24, Weizhen Wang, 添加mode 4功能，支持小推车数据机械编排和零速修正
* @internals   程序流程：\n
*              1. 显示菜单，用户选择功能模式\n
*              2. 根据模式执行相应的功能：\n
*                 - 标定模式：读取标定数据，计算误差参数，补偿数据\n
*                 - 粗对准模式：读取静态数据，计算姿态角，分析噪声\n
*                 - 机械编排模式：读取IMU数据，初始化状态，循环更新姿态/速度/位置，与参考值比较
*/
int main()
{
	int mode = -1;
	cout << "0加速度计标定 1陀螺仪标定 2粗对准 3示例数据进行机械编排 4小推车数据进行机械编排\n" << "请输入你需要解算的内容：";
	cin >> mode;
	if (!(mode == 0 || mode == 1 || mode == 2 || mode == 3 || mode == 4))cerr << "\n未识别解算内容“" << mode << "” 程序结束！\n";

	switch (mode)
	{
		/* 标定加速度计 */
	case 0:
	{
		// 标定文件路径
		string filename_1 = "Data\\Calibration\\x_up.ASC";
		string filename_2 = "Data\\Calibration\\x_down.ASC";
		string filename_3 = "Data\\Calibration\\y_up.ASC";
		string filename_4 = "Data\\Calibration\\y_down.ASC";
		string filename_5 = "Data\\Calibration\\z_up.ASC";
		string filename_6 = "Data\\Calibration\\z_down.ASC";
		string filename_c1 = "Data\\Calibration\\x_up.ASC";
		string filename_c2 = "Data\\Calibration\\x_down.ASC";
		string filename_c3 = "Data\\Calibration\\y_up.ASC";
		string filename_c4 = "Data\\Calibration\\y_down.ASC";
		string filename_c5 = "Data\\Calibration\\z_up.ASC";
		string filename_c6 = "Data\\Calibration\\z_down.ASC";

		string filesave = "Result\\Figure_Matlab\\AccCali\\AccCaliError.txt";
		string filesave_c1 = "Result\\Figure_Matlab\\AccCali\\x_up_Raw_Com.txt";
		string filesave_c2 = "Result\\Figure_Matlab\\AccCali\\x_down_Raw_Com.txt";
		string filesave_c3 = "Result\\Figure_Matlab\\AccCali\\y_up_Raw_Com.txt";
		string filesave_c4 = "Result\\Figure_Matlab\\AccCali\\y_down_Raw_Com.txt";
		string filesave_c5 = "Result\\Figure_Matlab\\AccCali\\z_up_Raw_Com.txt";
		string filesave_c6 = "Result\\Figure_Matlab\\AccCali\\z_down_Raw_Com.txt";

		AccCaliError error;

		ifstream file_1(filename_1), file_2(filename_2), file_3(filename_3), file_4(filename_4), file_5(filename_5), file_6(filename_6);
		ifstream file_c1(filename_c1), file_c2(filename_c2), file_c3(filename_c3), file_c4(filename_c4), file_c5(filename_c5), file_c6(filename_c6);
		ofstream file(filesave);
		ofstream file_s1(filesave_c1), file_s2(filesave_c2), file_s3(filesave_c3), file_s4(filesave_c4), file_s5(filesave_c5), file_s6(filesave_c6);
		if (!(file_1 || file_2 || file_3 || file_4 || file_5 || file_6 || file_c1 || file_c2 || file_c3 || file_c4 || file_c5 || file_c6)) { cerr << "\nCannot open the file to read rawdata!\n"; return -1; }
		if (!(file || file_s1 || file_s2 || file_s3 || file_s4 || file_s5 || file_s6)) { cerr << "\nCannot open the file to write rawdata!\n"; return -1; }

		//小推车实测数据标定
		ifstream IMU_file("Data\\MechanizationData\\group5.ASC");
		ofstream IMU_AccCali("Data\\MechanizationData\\group5_AccCali.txt");
		if(!IMU_file || !IMU_AccCali) { cerr << "\nCannot open the file \n"; return -1; }

		// 六位置法标定加速度计
		CalCaliBillMean(file_1, 0, error);	// 求平均
		CalCaliBillMean(file_2, 1, error);
		CalCaliBillMean(file_3, 2, error);
		CalCaliBillMean(file_4, 3, error);
		CalCaliBillMean(file_5, 4, error);
		CalCaliBillMean(file_6, 5, error);

		CalAcc(error, file);	// 求误差矩阵

		// 补偿误差
		CompenAcc(file_c1, error, file_s1);
		CompenAcc(file_c2, error, file_s2);
		CompenAcc(file_c3, error, file_s3);
		CompenAcc(file_c4, error, file_s4);
		CompenAcc(file_c5, error, file_s5);
		CompenAcc(file_c6, error, file_s6);
		CompenAcc(IMU_file, error, IMU_AccCali);

		// 关闭文件流
		file_1.close(); file_2.close(); file_3.close(); file_4.close(); file_5.close(); file_6.close();	
		file.close();
		file_c1.close(); file_c2.close(); file_c3.close(); file_c4.close(); file_c5.close(); file_c6.close();
		file_s1.close(); file_s2.close(); file_s3.close(); file_s4.close(); file_s5.close(); file_s6.close();
		IMU_file.close(); IMU_AccCali.close();

		break;
	}

		/* 标定陀螺 */
	case 1:
	{
		// 标定文件路径
		string filename_1 = "Data\\Calibration\\x_tl_1.ASC";
		string filename_2 = "Data\\Calibration\\x_tl_2.ASC";
		string filename_3 = "Data\\Calibration\\y_tl_1.ASC";
		string filename_4 = "Data\\Calibration\\y_tl_2.ASC";
		string filename_5 = "Data\\Calibration\\z_tl_1.ASC";
		string filename_6 = "Data\\Calibration\\z_tl_2.ASC";
		string filename_c1 = "Data\\Calibration\\x_tl_1.ASC";
		string filename_c2 = "Data\\Calibration\\x_tl_2.ASC";
		string filename_c3 = "Data\\Calibration\\y_tl_1.ASC";
		string filename_c4 = "Data\\Calibration\\y_tl_2.ASC";
		string filename_c5 = "Data\\Calibration\\z_tl_1.ASC";
		string filename_c6 = "Data\\Calibration\\z_tl_2.ASC";
		string filename_acc1 = "Data\\Calibration\\x_up.ASC";
		string filename_acc2 = "Data\\Calibration\\x_down.ASC";
		string filename_acc3 = "Data\\Calibration\\y_up.ASC";
		string filename_acc4 = "Data\\Calibration\\y_down.ASC";
		string filename_acc5 = "Data\\Calibration\\z_up.ASC";
		string filename_acc6 = "Data\\Calibration\\z_down.ASC";

		string filesave = "Result\\Figure_Matlab\\GyrCali\\GyroCalibration.txt";
		string filesave_s1 = "Result\\Figure_Matlab\\GyrCali\\x+360_Raw_Com.txt";
		string filesave_s2 = "Result\\Figure_Matlab\\GyrCali\\x-360_Raw_Com.txt";
		string filesave_s3 = "Result\\Figure_Matlab\\GyrCali\\y+360_Raw_Com.txt";
		string filesave_s4 = "Result\\Figure_Matlab\\GyrCali\\y-360_Raw_Com.txt";
		string filesave_s5 = "Result\\Figure_Matlab\\GyrCali\\z+360_Raw_Com.txt";
		string filesave_s6 = "Result\\Figure_Matlab\\GyrCali\\z-360_Raw_Com.txt";

		GyroCaliError error;

		// 打开文件流，检查文件是否存在
		ifstream file_1(filename_1), file_2(filename_2), file_3(filename_3), file_4(filename_4), file_5(filename_5), file_6(filename_6);
		ifstream file_c1(filename_c1), file_c2(filename_c2), file_c3(filename_c3), file_c4(filename_c4), file_c5(filename_c5), file_c6(filename_c6);
		ifstream file_acc1(filename_acc1), file_acc2(filename_acc2), file_acc3(filename_acc3), file_acc4(filename_acc4), file_acc5(filename_acc5), file_acc6(filename_acc6);
		ofstream file(filesave), file_s1(filesave_s1), file_s2(filesave_s2), file_s3(filesave_s3), file_s4(filesave_s4), file_s5(filesave_s5), file_s6(filesave_s6);
		if (!(file_1 || file_2 || file_3 || file_4 || file_5 || file_6 )) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
		if (!(file_c1 || file_c2 || file_c3 || file_c4 || file_c5 || file_c6)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
		if (!(file_acc1 || file_acc2 || file_acc3 || file_acc4 || file_acc5 || file_acc6)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
		if (!(file || file_s1 || file_s2 || file_s3 || file_s4 || file_s5 || file_s6)) { cerr << "Cannot open the file to write rawdata!\n"; return -1; }

		//小推车实测数据标定
		ifstream IMU_file("Data\\MechanizationData\\group5.ASC");
		ofstream IMU_gyrCali("Data\\MechanizationData\\group5_gyrCali.txt");
		if (!IMU_file || !IMU_gyrCali) { cerr << "\nCannot open the file \n"; return -1; }

		// 利用加速度计六位置法标定中采集的静态陀螺数据标定陀螺零偏 rad/s->deg/h
		CalGyrBias(file_acc1, file_acc2, 0, error);
		CalGyrBias(file_acc3, file_acc4, 1, error);
		CalGyrBias(file_acc5, file_acc6, 2, error);
		//cout << error.bias[0] << " " << error.bias[1] << " " << error.bias[2] << " " << endl;
		
		// 角位置法标定陀螺比力因子 rad->deg
		CalGyrScale(file_1, file_2, 0, error);
		CalGyrScale(file_4, file_3, 1, error);
		CalGyrScale(file_5, file_6, 2, error);
		//cout << error.scale[0] << " " << error.scale[1] << " " << error.scale[2] << " " << endl;

		// 输出陀螺标定结果
		file << fixed << setprecision(12);
		file << "%GyroBias_XYZ(deg/h):\n"
			<< error.bias[0] << "," << error.bias[1] << "," << error.bias[2] << endl;

		file << "%GyroScale_XYZ(deg):\n"
			<< error.scale[0] << "," << error.scale[1] << "," << error.scale[2] << endl;
		cout << "\n陀螺仪标定成功，结果已经保存在文件中！\n";

		CompenGyro(file_c1, 0, error, file_s1);
		CompenGyro(file_c2, 0, error, file_s2);
		CompenGyro(file_c3, 1, error, file_s3);
		CompenGyro(file_c4, 1, error, file_s4);
		CompenGyro(file_c5, 2, error, file_s5);
		CompenGyro(file_c6, 2, error, file_s6);
		CompenGyro(IMU_file, 3, error, IMU_gyrCali);


		// 关闭文件流
		file_1.close(); file_2.close(); file_3.close(); file_4.close(); file_5.close(); file_6.close();
		file_c1.close(); file_c2.close(); file_c3.close(); file_c4.close(); file_c5.close(); file_c6.close();
		file_acc1.close(); file_acc2.close(); file_acc3.close(); file_acc4.close(); file_acc5.close(); file_acc6.close();
		file.close(); 
		file_s1.close(); file_s2.close(); file_s3.close(); file_s4.close(); file_s5.close(); file_s6.close();
		IMU_file.close(); IMU_gyrCali.close();


		break;
	}

		/* 粗对准 */
	case 2:
	{
		// 打开数据文件
		string filename1 = "Data\\Align\\Align_30min.ASC";
		string filename2 = "Data\\Align\\Align_30min.ASC";
		string filename3 = "Data\\Align\\Align_30min.ASC";
		string filename4 = "Data\\Align\\Align_30min.ASC";

		// 打开数据保存文件
		string filename_save1 = "Result\\Figure_Matlab\\Align\\Align_Whole.txt";
		string filename_save2 = "Result\\Figure_Matlab\\Align\\Align_Second.txt";
		string filename_save3 = "Result\\Figure_Matlab\\Align\\Align_Epoch.txt";
		string filename_save4 = "Result\\Figure_Matlab\\Align\\Align_Deviation.txt";

		// 创建结构体
		ALIGNPOS pos1, pos2, pos3;

		// 打开文件流，检查文件是否存在
		ifstream file1(filename1), file2(filename2), file3(filename3), file4(filename4);
		ofstream file_save1(filename_save1), file_save2(filename_save2), file_save3(filename_save3), file_save4(filename_save4);
		if (!file1 || !file_save1 || !file_save2 || !file_save3 || !file_save4) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }

		// 计算姿态角
		CoarseAlign_Whole(file1,  pos1, file_save1);	        // 计算整段数据的姿态角并保存
		CoarseAlign_EverySecond(file2, pos2, pos1, file_save2);	// 计算每一秒的平均姿态角并保存
		CoarseAlign_EveryEpoch(file3, pos3, pos1, file_save3);	// 计算每个历元的姿态角并保存

		CalTimeNoise(file4, file_save4);	// 计算白噪声随时间变化大小

		file1.close(); file2.close(); file3.close(); file4.close();	// 关闭文件流
		file_save1.close();
		file_save2.close();
		file_save3.close();
		file_save4.close();

		break;
	}

	/* 使用示例数据进行惯导机械编排 */
	case 3:
	{
		//打开二进制文件
		ifstream IMU_example("Data\\MechanizationExampleData\\IMU.bin", ios::binary);
		ifstream INS_example("Data\\MechanizationExampleData\\PureINS.bin", ios::binary);
		ofstream result("Result\\Mechanization_Matlab\\MechanizationExample\\result.txt");             // 解算文件
		ofstream result_ref("Result\\Mechanization_Matlab\\MechanizationExample\\result_ref.txt");     // 参考文件
		ofstream result_diff("Result\\Mechanization_Matlab\\MechanizationExample\\result_diff.txt");   // 差值文件
		ofstream result_denu("Result\\Mechanization_Matlab\\MechanizationExample\\result_denu.txt");   // 轨迹文件


		if(!IMU_example || !INS_example){ cerr << "Cannot open the file to read rawdata!\n"; return -1; }
		if (!result || !result_ref || !result_diff || !result_denu) { cerr << "Cannot open the file to write result!\n"; return -1; }


		RAWDAT Rawdata_cur, Rawdata_prev1, Rawdata_prev2;
		InsState InsResult;         // 示例数据机械编排结果
		InsEpochState InsRef;       // 示例数据机械编排参考结果

		bool flag_imu = true,flag_ref=true;       // 第一次启动循环
		bool Is_prev1 = false;      // 上个历元是否有数据
		bool Is_prev2 = false;      // 上上个历元是否有数据
		while (flag_imu == true)
		{
			if (flag_imu) flag_imu = ReadExamplePureIMUData(IMU_example, Rawdata_cur);
			if (Rawdata_cur.time.Second < 91619.995) continue;       //小于示例数据的初始时间continue
			//cout << Rawdata.time.Second << "  " << Rawdata.gyr.X << "  " << Rawdata.gyr.Y << "  " << Rawdata.gyr.Z << "  "
			//	<< Rawdata.acc_v.X << "  " << Rawdata.acc_v.Y << "  " << Rawdata.acc_v.Z << "  " << endl;

			if (!Is_prev1)    // 判断上个历元的数据是否存下来了
			{
				Rawdata_prev1 = Rawdata_cur;
				Is_prev1 = true;  // 有数据了
				continue;
			}
			else if (!Is_prev2)   // 判断上上个历元的数据是否存下来了
			{
				Rawdata_prev2 = Rawdata_prev1;
				Rawdata_prev1 = Rawdata_cur;
				Is_prev2 = true;  // 上上个有数据了

				// 开始解算的时候初始化常量
				InsResult.prev1.time = Rawdata_prev1.time;
				InsResult.prev2.time = Rawdata_prev2.time;

				// 1. 速度初始化（不用初始化，开始的时候是0）
				// 2. 位置初始化
				InsResult.prev1.pos.latitude = InsResult.prev2.pos.latitude = initial_pos[0];
				InsResult.prev1.pos.longitude = InsResult.prev2.pos.longitude = initial_pos[1];
				InsResult.prev1.pos.H = InsResult.prev2.pos.H = initial_pos[2];
				// 3. 姿态初始化
				InsResult.prev1.atti.roll = InsResult.prev2.atti.roll = initial_atti[0];
				InsResult.prev1.atti.pitch = InsResult.prev2.atti.pitch = initial_atti[1];
				InsResult.prev1.atti.yaw = InsResult.prev2.atti.yaw = initial_atti[2];
				InsResult.prev1.quater.SetQbn(InsResult.prev1.atti);  // 设置上个四元数
				InsResult.prev2.quater.SetQbn(InsResult.prev2.atti);  // 设置上上个四元数
				continue;
			}


			/*** 开始推算 ***/
			InsResult.cur.time = Rawdata_cur.time;
			AttitudeUpdate(Rawdata_cur, Rawdata_prev1, InsResult);
			VelocityUpdate(Rawdata_cur, Rawdata_prev1, InsResult);
			PositionUpdate(Rawdata_cur, Rawdata_prev1, InsResult);

			// 与真值比较
			flag_ref = ReadExamplePureINSRef(INS_example, InsRef);
			double t_diff = InsResult.cur.time.Second - InsRef.time.Second;
			while(flag_ref==true && (t_diff > 1e-3))
			{
				flag_ref = ReadExamplePureINSRef(INS_example, InsRef);
				t_diff = InsResult.cur.time.Second - InsRef.time.Second;
			}
			if (!flag_ref)break;

			double Result_pos[3] = { InsResult.cur.pos.latitude  ,InsResult.cur.pos.longitude,InsResult.cur.pos.H };//单位：rad rad m
			double Ref_pos[3] = { InsRef.pos.latitude * Rad, InsRef.pos.longitude * Rad ,InsRef.pos.H };//单位：rad rad m
			double denu[3] = { 0.0 }, Result_denu[3] = { 0.0 }, Ref_denu[3] = { 0.0 };
			Comp_dEnu2(Ref_pos, Result_pos, R_WGS84, F_WGS84, denu);                // denu
			Comp_dEnu2(initial_pos, Result_pos, R_WGS84, F_WGS84, Result_denu);// 解算轨迹
			Comp_dEnu2(initial_pos, Ref_pos, R_WGS84, F_WGS84, Ref_denu);      // 参考轨迹
			double dpos[3] = { InsResult.cur.pos.latitude * Deg - InsRef.pos.latitude,InsResult.cur.pos.longitude * Deg - InsRef.pos.longitude,InsResult.cur.pos.H - InsRef.pos.H };
			double dvel[3] = { InsResult.cur.vel.Vn - InsRef.vel.Vn, InsResult.cur.vel.Ve - InsRef.vel.Ve, InsResult.cur.vel.Vd - InsRef.vel.Vd };
			if (InsResult.cur.atti.yaw < 0)InsResult.cur.atti.yaw += 2 * PAI;
			if (InsRef.atti.yaw < 0)InsRef.atti.yaw += 360;
			double datti[3] = { InsResult.cur.atti.roll * Deg - InsRef.atti.roll, InsResult.cur.atti.pitch * Deg - InsRef.atti.pitch , InsResult.cur.atti.yaw * Deg - InsRef.atti.yaw };


			//cout << fixed << setprecision(5) << InsResult.cur.time.Second << "  " << t_diff << "  "
			//	<< fixed << setprecision(8) << InsResult.cur.pos.latitude << "  " << InsResult.cur.pos.longitude << "  " << InsResult.cur.pos.H << "  "
			//	<< InsResult.cur.vel.Vn << "  " << InsResult.cur.vel.Ve << "  " << InsResult.cur.vel.Vd << "  "
			//	<< InsResult.cur.atti.roll << "  " << InsResult.cur.atti.pitch << "  " << InsResult.cur.atti.yaw << "  "
			//	<< dpos[0] << "  " << dpos[1] << "  " << dpos[2] << "  "
			//	<< dvel[0] << "  " << dvel[1] << "  " << dvel[2] << "  "
			//	<< datti[0] << "  " << datti[1] << "  " << datti[2] << "  " << endl;


			result << fixed << setprecision(5) << InsResult.cur.time.Second << "  "
				<< fixed << setprecision(12)
				<< InsResult.cur.pos.latitude * Deg << "  " << InsResult.cur.pos.longitude * Deg << "  " << InsResult.cur.pos.H << "  "
				<< InsResult.cur.vel.Vn << "  " << InsResult.cur.vel.Ve << "  " << InsResult.cur.vel.Vd << "  "
				<< InsResult.cur.atti.roll * Deg << "  " << InsResult.cur.atti.pitch * Deg << "  " << InsResult.cur.atti.yaw * Deg << "  " << endl;
			result_ref << fixed << setprecision(5) << InsRef.time.Second << "  "
				<< fixed << setprecision(12)
				<< InsRef.pos.latitude << "  " << InsRef.pos.longitude << "  " << InsRef.pos.H << "  "
				<< InsRef.vel.Vn << "  " << InsRef.vel.Ve << "  " << InsRef.vel.Vd << "  "
				<< InsRef.atti.roll << "  " << InsRef.atti.pitch << "  " << InsRef.atti.yaw << "  " << endl;
			result_diff << fixed << setprecision(5) << InsResult.cur.time.Second << "  "
				<< fixed << setprecision(12)
				<< dpos[0] << "  " << dpos[1] << "  " << dpos[2] << "  "
				<< dvel[0] << "  " << dvel[1] << "  " << dvel[2] << "  "
				<< datti[0] << "  " << datti[1] << "  " << datti[2] << "  "
				<< denu[0] << "  " << denu[1] << "  " << denu[2] << "  " << endl;
			result_denu << fixed << setprecision(5) << InsResult.cur.time.Second << "  "
				<< fixed << setprecision(12)
				<< Result_denu[0] << "  " << Result_denu[1] << "  " << Result_denu[2] << "  "
				<< Ref_denu[0] << "  " << Ref_denu[1] << "  " << Ref_denu[2] << "  " << endl;


			// 保存当前历元数据
			Rawdata_prev2 = Rawdata_prev1;
			Rawdata_prev1 = Rawdata_cur;
			InsResult.prev2 = InsResult.prev1;
			InsResult.prev1 = InsResult.cur;

		}
		IMU_example.close();
		INS_example.close();
		result.close();
		result_ref.close();
		result_diff.close();
		result_denu.close();
		break;
	}

	case 4:
	{
		int Is_Zero = 0;   // 是否启用零速修正
		cout << "0不启用零速修正 1启用零速修正\n" << "请输入是否启用零速修正：";
		cin >> Is_Zero;
		if (!(Is_Zero == 0 || Is_Zero == 1 ))cerr << "\n未识别是否启用零速修正“" << Is_Zero << "” 程序结束！\n";

		//打开文件
		ifstream IMU_file("Data\\MechanizationData\\group5.ASC");
		ifstream INS_file("Data\\MechanizationData\\IEproject5.ref");
		ifstream IMU_Align("Data\\MechanizationData\\group5_Align.ASC");
		ofstream result("Result\\Mechanization_Matlab\\Mechanization\\result.txt");             // 解算文件
		ofstream result_Align("Result\\Mechanization_Matlab\\Mechanization\\result_Align.txt"); 
		ofstream result_ref("Result\\Mechanization_Matlab\\Mechanization\\result_ref.txt");     // 参考文件
		ofstream result_diff("Result\\Mechanization_Matlab\\Mechanization\\result_diff.txt");   // 差值文件
		ofstream result_denu("Result\\Mechanization_Matlab\\Mechanization\\result_denu.txt");   // 轨迹文件
		//可对实验采集的陀螺仪进行标定和补偿
		//ifstream IMU_file("Data\\MechanizationData\\group5_gyrCali.ASC");
		//ifstream INS_file("Data\\MechanizationData\\IEproject5.ref");
		//ifstream IMU_Align("Data\\MechanizationData\\group5_Align.ASC");
		//ofstream result("Result\\Mechanization_Matlab\\Mechanization_gyrCali\\result.txt");             // 解算文件
		//ofstream result_Align("Result\\Mechanization_Matlab\\Mechanization_gyrCali\\result_Align.txt");
		//ofstream result_ref("Result\\Mechanization_Matlab\\Mechanization_gyrCali\\result_ref.txt");     // 参考文件
		//ofstream result_diff("Result\\Mechanization_Matlab\\Mechanization_gyrCali\\result_diff.txt");   // 差值文件
		//ofstream result_denu("Result\\Mechanization_Matlab\\Mechanization_gyrCali\\result_denu.txt");   // 轨迹文件

		if (!IMU_file ||!IMU_Align || !INS_file) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
		if (!result || !result_Align || !result_ref || !result_diff ||!result_denu) { cerr << "Cannot open the file to write result!\n"; return -1; }

		/*** 解析粗对准确定初始姿态 ***/
		ALIGNPOS pos;
		CoarseAlign_Mechanization(IMU_Align, pos, result_Align);	        // 计算整段数据的姿态角并保存

		/*** 开始捷联惯导机械编排 ***/
		RAWDAT Rawdata_cur, Rawdata_prev1, Rawdata_prev2;
		InsState InsResult;         // 机械编排结果
		InsEpochState InsRef;       // 机械编排参考结果
		TimeIntervalsArray ZeroSpeed;   // 零速修正时间区间

		bool flag_imu = true, flag_ref = true;       // 第一次启动循环
		bool Is_prev1 = false;      // 上个历元是否有数据
		bool Is_prev2 = false;      // 上上个历元是否有数据
		// 读一次参考文件
		flag_ref = ReadPureINSRef(INS_file, InsRef);
		while (flag_imu == true)
		{
			if (flag_imu) flag_imu = ReadPureIMUData(IMU_file, Mechanization, Rawdata_cur);
			if (Rawdata_cur.time.Second <98567.980) continue;
			//if (Rawdata_cur.time.Second < 98129.000) continue;       //小于示例数据的初始时间continue
			//cout << Rawdata.time.Second << "  " << Rawdata.gyr.X << "  " << Rawdata.gyr.Y << "  " << Rawdata.gyr.Z << "  "
			//	<< Rawdata.acc_v.X << "  " << Rawdata.acc_v.Y << "  " << Rawdata.acc_v.Z << "  " << endl;

			if (!Is_prev1)    // 判断上个历元的数据是否存下来了
			{
				Rawdata_prev1 = Rawdata_cur;
				Is_prev1 = true;  // 有数据了
				continue;
			}
			else if (!Is_prev2)   // 判断上上个历元的数据是否存下来了
			{
				Rawdata_prev2 = Rawdata_prev1;
				Rawdata_prev1 = Rawdata_cur;
				Is_prev2 = true;  // 上上个有数据了

				// 开始解算的时候初始化常量
				InsResult.prev1.time = Rawdata_prev1.time;
				InsResult.prev2.time = Rawdata_prev2.time;

				// 1. 速度初始化（不用初始化，开始的时候是0）
				// 2. 位置初始化
				InsResult.prev1.pos.latitude = InsResult.prev2.pos.latitude = ours_initial_pos[0];
				InsResult.prev1.pos.longitude = InsResult.prev2.pos.longitude = ours_initial_pos[1];
				InsResult.prev1.pos.H = InsResult.prev2.pos.H = ours_initial_pos[2];
				// 3. 姿态初始化
				InsResult.prev1.atti.roll = InsResult.prev2.atti.roll = ours_initial_atti[0];
				InsResult.prev1.atti.pitch = InsResult.prev2.atti.pitch = ours_initial_atti[1];
				InsResult.prev1.atti.yaw = InsResult.prev2.atti.yaw = ours_initial_atti[2];
				//InsResult.prev1.atti.roll = InsResult.prev2.atti.roll = pos.roll;
				//InsResult.prev1.atti.pitch = InsResult.prev2.atti.pitch = pos.pitch;
				//InsResult.prev1.atti.yaw = InsResult.prev2.atti.yaw = pos.yaw;
				InsResult.prev1.quater.SetQbn(InsResult.prev1.atti);  // 设置上个四元数
				InsResult.prev2.quater.SetQbn(InsResult.prev2.atti);  // 设置上上个四元数
				continue;
			}


			/*** 开始推算 ***/
			InsResult.cur.time = Rawdata_cur.time;
			VelocityUpdate(Rawdata_cur, Rawdata_prev1, InsResult);
			if (Is_Zero == 1)    // 是否进行零速修正
			{
				// 零速修正
				for (int i = 0; i < zero_time_intervals_num; i++)
				{
					if (Rawdata_cur.time.Second > ZeroSpeed.getInterval(i).end)
					{
						ZeroSpeed.getInterval(i).used = true;   // 已经进行过零速修正的内容
						continue;
					}

					if (i < zero_time_intervals_num - 1 &&
						Rawdata_cur.time.Second > ZeroSpeed.getInterval(i).end &&
						Rawdata_cur.time.Second < ZeroSpeed.getInterval(i + 1).start) // 如果在某两区间的中间，则直接退出
						break;

					if (Rawdata_cur.time.Second >= ZeroSpeed.getInterval(i).start &&
						Rawdata_cur.time.Second <= ZeroSpeed.getInterval(i).end)  // 在某区间内
					{
						InsResult.cur.vel.Vn = 0.0;
						InsResult.cur.vel.Ve = 0.0;
						InsResult.cur.vel.Vd = 0.0;
						break;
					}
				}
			}

			PositionUpdate(Rawdata_cur, Rawdata_prev1, InsResult);
			AttitudeUpdate(Rawdata_cur, Rawdata_prev1, InsResult);


			/*** 与真值比较 ***/
			double t_diff = InsResult.cur.time.Second - InsRef.time.Second;// 时间同步
			while (flag_ref == true && (fabs(t_diff) > 0.05))
			{
				flag_ref = ReadPureINSRef(INS_file, InsRef);
				t_diff = InsResult.cur.time.Second - InsRef.time.Second;
			}
			if (!flag_ref) break;

			double Result_pos[3] = { InsResult.cur.pos.latitude  ,InsResult.cur.pos.longitude,InsResult.cur.pos.H };//单位：rad rad m
			double Ref_pos[3] = { InsRef.pos.latitude * Rad, InsRef.pos.longitude * Rad ,InsRef.pos.H };//单位：rad rad m
			double denu[3] = { 0.0 }, Result_denu[3] = { 0.0 }, Ref_denu[3] = { 0.0 };
			Comp_dEnu2(Ref_pos, Result_pos, R_WGS84, F_WGS84, denu);                // denu
			Comp_dEnu2(ours_initial_pos, Result_pos, R_WGS84, F_WGS84, Result_denu);// 解算轨迹
			Comp_dEnu2(ours_initial_pos, Ref_pos, R_WGS84, F_WGS84, Ref_denu);      // 参考轨迹
			double dpos[3] = { InsResult.cur.pos.latitude * Deg - InsRef.pos.latitude,InsResult.cur.pos.longitude * Deg - InsRef.pos.longitude,InsResult.cur.pos.H - InsRef.pos.H };
			double dvel[3] = { InsResult.cur.vel.Vn - InsRef.vel.Vn, InsResult.cur.vel.Ve - InsRef.vel.Ve, InsResult.cur.vel.Vd - InsRef.vel.Vd };
			if (InsResult.cur.atti.yaw < 0)InsResult.cur.atti.yaw += 2 * PAI;
			if (InsRef.atti.yaw < 0)InsRef.atti.yaw += 360;
			double datti[3] = { InsResult.cur.atti.roll * Deg - InsRef.atti.roll, InsResult.cur.atti.pitch * Deg - InsRef.atti.pitch , InsResult.cur.atti.yaw * Deg - InsRef.atti.yaw };

			//cout << fixed << setprecision(5) << InsResult.cur.time.Second << "  " << t_diff << "  "
			//	<< fixed << setprecision(12) << InsResult.cur.pos.latitude << "  " << InsResult.cur.pos.longitude << "  " << InsResult.cur.pos.H << "  "
			//	<< InsResult.cur.vel.Vn << "  " << InsResult.cur.vel.Ve << "  " << InsResult.cur.vel.Vd << "  "
			//	<< InsResult.cur.atti.roll << "  " << InsResult.cur.atti.pitch << "  " << InsResult.cur.atti.yaw << "  "
			//	<< dpos[0] << "  " << dpos[1] << "  " << dpos[2] << "  "
			//	<< dvel[0] << "  " << dvel[1] << "  " << dvel[2] << "  "
			//	<< datti[0] << "  " << datti[1] << "  " << datti[2] << "  " << endl;

			result<< fixed << setprecision(5) << InsResult.cur.time.Second << "  "
				<< fixed << setprecision(12)
				<< InsResult.cur.pos.latitude * Deg << "  " << InsResult.cur.pos.longitude * Deg << "  " << InsResult.cur.pos.H << "  "
				<< InsResult.cur.vel.Vn << "  " << InsResult.cur.vel.Ve << "  " << InsResult.cur.vel.Vd << "  "
				<< InsResult.cur.atti.roll * Deg << "  " << InsResult.cur.atti.pitch * Deg << "  " << InsResult.cur.atti.yaw * Deg << "  " << endl;
			result_ref << fixed << setprecision(5) << InsRef.time.Second << "  "
				<< fixed << setprecision(12)
				<< InsRef.pos.latitude << "  " << InsRef.pos.longitude << "  " << InsRef.pos.H << "  "
				<< InsRef.vel.Vn << "  " << InsRef.vel.Ve << "  " << InsRef.vel.Vd << "  "
				<< InsRef.atti.roll << "  " << InsRef.atti.pitch << "  " << InsRef.atti.yaw << "  " << endl;
			result_diff<< fixed << setprecision(5) << InsResult.cur.time.Second << "  "
				<< fixed << setprecision(12) 
				<< dpos[0] << "  " << dpos[1] << "  " << dpos[2] << "  "
				<< dvel[0] << "  " << dvel[1] << "  " << dvel[2] << "  "
				<< datti[0] << "  " << datti[1] << "  " << datti[2] << "  "
				<< denu[0] << "  " << denu[1] << "  " << denu[2] << "  "<< endl;
			result_denu << fixed << setprecision(5) << InsResult.cur.time.Second << "  "
				<< fixed << setprecision(12)
				<< Result_denu[0] << "  " << Result_denu[1] << "  " << Result_denu[2] << "  "
				<< Ref_denu[0] << "  " << Ref_denu[1] << "  " << Ref_denu[2] << "  " << endl;


			// 保存当前历元数据
			Rawdata_prev2 = Rawdata_prev1;
			Rawdata_prev1 = Rawdata_cur;
			InsResult.prev2 = InsResult.prev1;
			InsResult.prev1 = InsResult.cur;

		}
		IMU_file.close();
		INS_file.close();
		IMU_Align.close();
		result.close();
		result_Align.close();
		result_ref.close(); 
		result_diff.close();
		result_denu.close();

		break;
	}


	}

    return 0;
}
