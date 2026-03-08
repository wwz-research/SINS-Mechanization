/**
* @file     ReadFile.cpp
* @brief    IMU ASC文件读取与解析的实现文件
* @details  本文件实现了从不同 IMU 设备输出的ASC文件中读取一行观测数据，\n
*           并解析为统一的 RAWDAT 结构体，包括时间标签、加速度计观测量和陀螺仪观测量，\n
*           函数实现需配合 ReadFile.h 中的函数声明一起使用，注释格式遵循项目统一规范。
* @author   Weizhen Wang. Email: wwz131407080103@qq.com
* @date     2025/11/01
* @version  1.1.20251101
* @par      This file is a part of SINS (Strapdown Inertial Navigation System).
* @par      History:
*           2025/11/01, Weizhen Wang, Create
*           2025/11/02,Weizhen Wang, 添加函数 ReadFile_Ali(ifstream&, DeviceType, RAWDAT&) 用于读取粗对准文件数据并写入 Rawdata 结构体。
*			2025/12/21, Weizhen Wang, 添加函数 ReadExamplePureIMUData(ifstream& file, RAWDAT& Rawdata) 用于读取示例数据
*			2025/12/21, Weizhen Wang, 添加函数 ReadExamplePureINSRef(ifstream& file, InsEpochState& InsRef) 用于读取示例数据的机械编排参考结果
*			2025/12/24, Weizhen Wang, 添加函数 ReadPureIMUData(ifstream& file, const DeviceType device, RAWDAT& Rawdata) 用于读取小推车的IMU数据
*			2025/12/24, Weizhen Wang, 添加函数 ReadPureINSRef(ifstream& file, InsEpochState& InsRef) 用于读取小推车的机械编排参考结果
*/

#include"ReadFile.h"

/**
* @param[in]   file      ifstream&   输入IMUASC文件文件流.
* @param[in]   device    DeviceType  设备类型（仅当为XWGI时本函数才生效）.
* @param[out]  Rawdata   RAWDAT&     输出一行解析后的IMU数据（时间、加速度计和陀螺仪）.
* @return      bool      返回是否读取并解析成功，true表示成功，false表示失败.
* @note        函数用于从标定设备（XWGI）输出的ASC文件中读取一行IMU原始数据，\n
*              按照固定格式解析出GPS周、周内秒、加速度计和陀螺仪原始量，并根据标定系数完成单位换算和符号调整。\n
*              当设备类型不是XWGI、当前行为空或格式不符合预期时，函数返回false。
* @par History:
*              
* @internals   内部实现采用std::getline读取整行字符串，利用std::stringstream按';'和','分隔符逐段解析，\n
*              跳过无关字段后依次填充Rawdata.time、Rawdata.acc和Rawdata.gyr相关成员，并应用AS_XWGI、GS_XWGI和r_XWGI进行尺度变换。
*/
bool ReadFile_Cali(ifstream& file, const DeviceType device, RAWDAT& Rawdata)
{
	if (device != Calibraton) return false;// 实验所用的标定设备

	// 读取文件数据
	string line;
	if (getline(file, line))
	{
		if (line.empty()) return false;

		stringstream ss(line);       // 使用字符串流，逐个字段解析
		string part;

		// 跳过%RAWIMUSA,2386,99960.820;
		getline(ss, part, ';');

		// 读取周 周内秒
		getline(ss, part, ',');	// GPS周
		Rawdata.time.Week = stoi(part);
		getline(ss, part, ',');	// GPS周秒
		Rawdata.time.Second = stod(part);

		// 跳过00000077,
		getline(ss, part, ',');

		// 读取加速度计数据
		getline(ss, part, ',');	// Z
		Rawdata.acc.Z = stod(part) * AS_Calibraton * r_Calibraton;
		getline(ss, part, ',');	// -Y                        //  轴给出的为负方向需要转号Y
		Rawdata.acc.Y = -stod(part) * AS_Calibraton * r_Calibraton;
		getline(ss, part, ',');	// X
		Rawdata.acc.X = stod(part) * AS_Calibraton * r_Calibraton;

		// 读取陀螺仪数据：角速度增量（rad/s）、角度增量（rad）
		getline(ss, part, ',');	// Z
		Rawdata.gyr_v.Z = stod(part) * GS_Calibraton * r_Calibraton;
		Rawdata.gyr.Z = stod(part) * GS_Calibraton;
		getline(ss, part, ',');	// -Y
		Rawdata.gyr_v.Y = -stod(part) * GS_Calibraton * r_Calibraton;
		Rawdata.gyr.Y = -stod(part) * GS_Calibraton;
		getline(ss, part, '*');	// X
		Rawdata.gyr_v.X = stod(part) * GS_Calibraton * r_Calibraton;
		Rawdata.gyr.X = stod(part) * GS_Calibraton;

		return true;	// 读取一行成功
	}
	return false;	// 读取一行失败则返回false
}

/**
* @param[in]   file      ifstream&   输入IMUASC文件流.
* @param[in]   device    DeviceType  设备类型（仅当为NSC时本函数才生效）.
* @param[out]  Rawdata   RAWDAT&     输出一行解析后的原始IMU数据（时间、加速度计和陀螺仪）.
* @return      bool      返回是否读取并解析成功，true表示成功，false表示失败.
* @note        函数用于从粗对准设备（NSC）输出的ASC文件中读取一行IMU原始数据，\n
*              按照预定格式解析出GPS周、周内秒以及三轴加速度和角增量信息，并根据NSC对应的比例因子进行单位换算和符号修正。\n
*              当设备类型不是NSC、当前行为空或数据格式不符合要求时，函数返回false。
* @par History:
*              
* @internals   内部实现与ReadFile_Cali类似，先跳过前导标识字段，再解析时间标签和原始观测值，\n
*              对Y轴量进行符号取反，对各通道乘以AS_NSC和GS_NSC完成归一化和物理量转换。
*/
bool ReadFile_Ali(ifstream& file, const DeviceType device, RAWDAT& Rawdata)
{
	if (device !=  Align) return false;// 实验所用的粗对准设备

	// 读取文件数据
	string line;
	if (getline(file, line))
	{
		if (line.empty())return false;	// 行为空时继续循环

		stringstream ss(line);	// 使用字符串流 逐个字段解析
		string part;

		// 跳过%RAWIMUSA,2387,558507.850;
		getline(ss, part, ';');

		// 跳过04,26,
		getline(ss, part, ',');
		getline(ss, part, ',');

		// 读取周 周内秒
		getline(ss, part, ',');	// 周
		Rawdata.time.Week = stoi(part);
		getline(ss, part, ',');	// 周内秒
		Rawdata.time.Second = stod(part);

		// 跳过2dcf0000,
		getline(ss, part, ',');

		// 读取加速度计数据
		getline(ss, part, ',');	// Z
		Rawdata.acc.Z = stod(part) * AS_Align * r_Align;
		getline(ss, part, ',');	// -Y
		Rawdata.acc.Y = -stod(part) * AS_Align * r_Align;
		getline(ss, part, ',');	// X
		Rawdata.acc.X = stod(part) * AS_Align * r_Align;

		// 读取陀螺仪数据
		getline(ss, part, ',');	// Z(弧度)
		Rawdata.gyr_v.Z = stod(part) * r_Align * GS_Align;
		getline(ss, part, ',');	// -Y(弧度)
		Rawdata.gyr_v.Y = -stod(part) * r_Align * GS_Align;
		getline(ss, part, '*');	// X(弧度)
		Rawdata.gyr_v.X = stod(part) * r_Align * GS_Align;

		return true;	// 读取一行成功
	}
	return false;	// 读取一行失败则返回false
}

/**
* @param[in]   file      ifstream&      已打开的二进制文件流
* @param[out]  Rawdata   RAWDAT&       读取一个历元的IMU数据
* @return      bool  返回是否读取成功，true表示成功，false表示失败
* @note        读取一个历元的示例数据IMU数据。\n
*              从二进制文件中读取时间戳、陀螺仪和加速度计数据。
* @par History:
*              2025/12/21, Weizhen Wang, Create
* @internals   从二进制文件中读取7个double类型数据：时间戳、陀螺仪XYZ、加速度计XYZ。\n
*              映射到Rawdata结构体的相应字段，其他字段初始化为0.0。
*/
bool ReadExamplePureIMUData(ifstream& file, RAWDAT& Rawdata)
{

	double tempData[7];  // TimeStamp, Gyr.X/Y/Z, Acc.X/Y/Z
	file.read(reinterpret_cast<char*>(tempData), 7 * sizeof(double));

	if (file.gcount() != 7 * sizeof(double)) {return false;}

	// 映射数据
	Rawdata.time.Week = 0;
	Rawdata.time.Second = tempData[0];
	Rawdata.gyr.X = tempData[1];
	Rawdata.gyr.Y = tempData[2];
	Rawdata.gyr.Z = tempData[3];
	Rawdata.acc_v.X = tempData[4];
	Rawdata.acc_v.Y = tempData[5];
	Rawdata.acc_v.Z = tempData[6];

	// 将其他字段初始化，避免上一轮更新带来的影响
	Rawdata.acc.X = Rawdata.acc.Y = Rawdata.acc.Z = 0.0;
	Rawdata.gyr_v.X = Rawdata.gyr_v.Y = Rawdata.gyr_v.Z = 0.0;

	return true;
}

/**
* @param[in]   file      ifstream&      已打开的二进制文件流
* @param[out]  InsRef    InsEpochState& 读取一个历元的机械编排参考结果
* @return      bool  返回是否读取成功，true表示成功，false表示失败
* @note        读取一个历元的示例数据机械编排参考结果。\n
*              从二进制文件中读取时间戳、位置、速度、姿态等完整状态信息。
* @par History:
*              2025/12/21, Weizhen Wang, Create
* @internals   从二进制文件中读取10个double类型数据：时间戳、位置（纬度、经度、高度，单位：deg）、\n
*              速度（Vn、Ve、Vd）、姿态（横滚、俯仰、航向，单位：deg）。\n
*              映射到InsRef结构体的相应字段，其他字段初始化为0.0。
*/
bool ReadExamplePureINSRef(ifstream& file, InsEpochState& InsRef)
{

	double tempData[10];  // TimeStamp, Gyr.X/Y/Z, Acc.X/Y/Z
	file.read(reinterpret_cast<char*>(tempData), 10 * sizeof(double));

	if (file.gcount() != 10 * sizeof(double)) { return false; }

	// 映射数据
	InsRef.time.Week = 0;
	InsRef.time.Second = tempData[0];
	InsRef.pos.latitude = tempData[1];   // deg
	InsRef.pos.longitude = tempData[2];  // deg
	InsRef.pos.H = tempData[3];
	InsRef.vel.Vn = tempData[4];
	InsRef.vel.Ve = tempData[5];
	InsRef.vel.Vd = tempData[6];
	InsRef.atti.roll = tempData[7];    // deg
	InsRef.atti.pitch = tempData[8];   // deg
	InsRef.atti.yaw = tempData[9];     // deg


	// 将其他字段初始化，避免上一轮更新带来的影响
	InsRef.denu.dE = InsRef.denu.dN = InsRef.denu.dU = 0.0;
	InsRef.quater.q[0] = InsRef.quater.q[1] = InsRef.quater.q[2] = InsRef.quater.q[3] = 0.0;

	return true;
}


/**
* @param[in]   file      ifstream&      已打开的ASC文件流
* @param[in]   device    DeviceType     设备类型，必须为Mechanization时才有效
* @param[out]  Rawdata   RAWDAT&       读取一个历元的IMU数据
* @return      bool  返回是否读取成功，true表示成功，false表示失败
* @note        读取一个历元的小推车IMU数据。\n
*              从ASC格式文件中读取数据，注意坐标系变换。
* @par History:
*              2025/12/24, Weizhen Wang, Create
* @internals   从ASC文件中读取一行数据，解析GPS时间和IMU数据。\n
*              进行坐标系变换：加速度计Z轴取反，X和Y轴交换；陀螺仪Z轴取反，X和Y轴交换。\n
*              使用AS_Mechanization和GS_Mechanization进行单位转换。
*/
bool ReadPureIMUData(ifstream& file, const DeviceType device, RAWDAT& Rawdata)
{
	if (device != Mechanization) return false;// 实验所用的标定设备

	// 读取文件数据
	string line;
	if (getline(file, line))
	{
		if (line.empty()) return false;

		stringstream ss(line);       // 使用字符串流，逐个字段解析
		string part;

		// 跳过%RAWIMUSA,2386,99960.820;
		getline(ss, part, ';');

		// 读取周 周内秒
		getline(ss, part, ',');	// GPS周
		Rawdata.time.Week = stoi(part);
		getline(ss, part, ',');	// GPS周秒
		Rawdata.time.Second = stod(part);

		// 跳过00000077,
		getline(ss, part, ',');

		// 读取加速度计数据
		getline(ss, part, ',');	// Z                          // 轴系调整！
		Rawdata.acc.Z = -stod(part) * AS_Mechanization * r_Mechanization;
		Rawdata.acc_v.Z = -stod(part) * AS_Mechanization;
		getline(ss, part, ',');	// -Y                        //  轴给出的为负方向需要转号Y
		Rawdata.acc.X = -stod(part) * AS_Mechanization * r_Mechanization;
		Rawdata.acc_v.X = -stod(part) * AS_Mechanization;
		getline(ss, part, ',');	// X
		Rawdata.acc.Y = stod(part) * AS_Mechanization * r_Mechanization;
		Rawdata.acc_v.Y = stod(part) * AS_Mechanization;

		// 读取陀螺仪数据：角速度增量（rad/s）、角度增量（rad）
		getline(ss, part, ',');	// Z
		Rawdata.gyr_v.Z = -stod(part) * GS_Mechanization * r_Mechanization;
		Rawdata.gyr.Z = -stod(part) * GS_Mechanization;
		getline(ss, part, ',');	// -Y
		Rawdata.gyr_v.X = -stod(part) * GS_Mechanization * r_Mechanization;
		Rawdata.gyr.X = -stod(part) * GS_Mechanization;
		getline(ss, part, '*');	// X
		Rawdata.gyr_v.Y = stod(part) * GS_Mechanization * r_Mechanization;
		Rawdata.gyr.Y = stod(part) * GS_Mechanization;


		return true;	// 读取一行成功
	}
	return false;	// 读取一行失败则返回false
}

/**
* @param[in]   file      ifstream&      已打开的文件流
* @param[out]  InsRef    InsEpochState& 读取一个历元的机械编排参考结果
* @return      bool  返回是否读取成功，true表示成功，false表示失败
* @note        读取小推车机械编排参考结果。\n
*              从文本文件中读取一行参考数据，包括GPS时间、位置、速度和姿态信息。
* @par History:
*              2025/12/24, Weizhen Wang, Create
* @internals   从文本文件中读取一行数据，跳过注释行和空行。\n
*              解析格式：Week GPSTime Latitude Longitude H-El ... VEast VNorth VUp Heading Pitch Rol\n
*              跳过前13个字段，读取速度（VEast、VNorth、VUp）和姿态（Heading、Pitch、Rol）。\n
*              注意VUp需要取反转换为Vd。
*/
bool ReadPureINSRef(ifstream& file, InsEpochState& InsRef)
{
	string line;
	while (getline(file, line))
	{
		// 跳过空行
		if (line.empty()) continue;

		// 跳过非数字开头的行（头文件）
		if (!std::isdigit(line[0])) continue;

		std::stringstream ss(line);
		//Week    GPSTime       Latitude      Longitude        H-El
		double week, sec, lat, lon, h;
		ss >> week >> sec >> lat >> lon >> h;
		// 跳过 13 个不用的字段
		// Q AmbStatus   Date    GPSTime      X-ECEF     Y-ECEF    Z-ECEF   X-LL   Y-LL     Z-LL   VX-ECEF   VY-ECEF   VZ-ECEF   
		std::string dummy;
		for (int i = 0; i < 13; ++i)
		{
			ss >> dummy;
		}
		//  VEast    VNorth       VUp       Heading          Pitch           Rol
		double VEast, VNorth, VUp, Heading, Pitch, Rol;
		ss >> VEast >> VNorth >> VUp >> Heading >> Pitch >> Rol;

		InsRef.time.Week = week;
		InsRef.time.Second = sec;
		InsRef.pos.latitude = lat;   // deg
		InsRef.pos.longitude = lon;  // deg
		InsRef.pos.H = h;
		InsRef.vel.Vn = VNorth;
		InsRef.vel.Ve = VEast;
		InsRef.vel.Vd = -VUp;
		InsRef.atti.roll = Rol;
		InsRef.atti.pitch = Pitch;
		InsRef.atti.yaw = Heading;
		return true;	// 读取一行成功
	}
	return false;	// 读取一行失败则返回false
}