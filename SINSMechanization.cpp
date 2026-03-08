#include"SINSMechanization.h"

/**
* @param[in]   Rawdata_cur  const RAWDAT&     当前历元原始数据
* @param[in]   Rawdata_pre  const RAWDAT&     前一个历元原始数据
* @param[in,out] InsState   InsState&        惯导状态（输入前一历元状态，输出当前历元状态）
* @return      bool  返回是否更新成功，true表示成功，false表示失败
* @note        姿态更新函数，使用四元数方法更新当前历元的姿态。\n
*              计算载体系和导航系的旋转，通过四元数乘法组合得到当前姿态四元数，然后转换为姿态角。
* @par History:
*              2025/12/24, Weizhen Wang, Create
* @internals   计算步骤：\n
*              1. 计算载体系有效旋转矢量Fai_k（考虑圆锥效应补偿）\n
*              2. 根据Fai_k计算载体系四元数增量q_b\n
*              3. 计算导航系有效旋转矢量Theta_k（地球自转和导航系相对地球的旋转）\n
*              4. 根据Theta_k计算导航系四元数增量q_n\n
*              5. 通过四元数乘法组合：q_cur = q_n ? q_prev ? q_b\n
*              6. 归一化四元数并转换为姿态角
*/
bool AttitudeUpdate(const RAWDAT Rawdata_cur,const RAWDAT Rawdata_pre, InsState& InsState)
{
	// 1. 等效旋转矢量法更新b系
	double gyr_cur[3] = { Rawdata_cur.gyr.X,Rawdata_cur.gyr.Y,Rawdata_cur.gyr.Z };
	double gyr_pre[3] = { Rawdata_pre.gyr.X,Rawdata_pre.gyr.Y,Rawdata_pre.gyr.Z };
	double gyr_product[3] = { 0.0 };
	CrossDot(3, 3, gyr_pre, gyr_cur, gyr_product);
	double Fai_k[3] = { 0.0 };
	for (int i = 0; i < 3; i++) { Fai_k[i] = gyr_cur[i] + gyr_product[i] / 12.0; }

	QUATER q_b;
	q_b.CalQ_b(Fai_k);

	// 2. 等效旋转矢量法更新n系
	double R_M = R_WGS84 * (1 - E2_WGS84) / sqrt(pow(1 - E2_WGS84 * sin(InsState.prev1.pos.latitude) * sin(InsState.prev1.pos.latitude), 3));
	double R_N = R_WGS84 / sqrt(1 - E2_WGS84 * sin(InsState.prev1.pos.latitude) * sin(InsState.prev1.pos.latitude));
	double wenn_pre[3] = { InsState.prev1.vel.Ve / (R_N + InsState.prev1.pos.H),-InsState.prev1.vel.Vn / (R_M + InsState.prev1.pos.H),-InsState.prev1.vel.Ve * tan(InsState.prev1.pos.latitude) / (R_N + InsState.prev1.pos.H) };
	double wien_pre[3] = { we * cos(InsState.prev1.pos.latitude),0.0,-we * sin(InsState.prev1.pos.latitude) };
	double deltaT = (Rawdata_cur.time.Week - Rawdata_pre.time.Week)*604800.0 + (Rawdata_cur.time.Second - Rawdata_pre.time.Second);
	double Theta_k[3] = { 0.0 };
	for (int j = 0; j < 3; j++) { Theta_k[j] = (wenn_pre[j] + wien_pre[j]) * deltaT; }
	
	QUATER q_n;
	q_n.CalQ_n(Theta_k);

	// 3. 计算当前姿态四元数
	QUATER q_middle,q_cur;
	QuaternionMultiply(q_n, InsState.prev1.quater, q_middle);// ！！！反嘞
	QuaternionMultiply(q_middle, q_b, q_cur);

	// 4. 归一化姿态四元数，计算当前姿态
	double norm = sqrt(q_cur.q[0] * q_cur.q[0] + q_cur.q[1] * q_cur.q[1] + q_cur.q[2] * q_cur.q[2] + q_cur.q[3] * q_cur.q[3]);
	if (norm != 0.0) { for (int k = 0; k < 4; k++) { InsState.cur.quater.q[k] = q_cur.q[k] / norm; } }
	CalPostureWithQuaternion(InsState.cur.quater, InsState.cur.atti);//由四元数计算姿态角

	return true;
}

/**
* @param[in]   Rawdata_cur  const RAWDAT&     当前历元原始数据
* @param[in]   Rawdata_pre  const RAWDAT&     前一个历元原始数据
* @param[in,out] InsState   InsState&        惯导状态（输入前一历元状态，输出当前历元状态）
* @return      bool  返回是否更新成功，true表示成功，false表示失败
* @note        速度更新函数，使用比力积分更新当前历元的速度。\n
*              考虑哥氏效应、重力效应和划桨效应补偿。
* @par History:
*              2025/12/24, Weizhen Wang, Create
* @internals   计算步骤：\n
*              1. 外推计算中间时刻的角速度、速度和重力\n
*              2. 计算哥氏效应修正项（2*wien + wenn）\n
*              3. 计算旋转效应修正矩阵（I - Theta×/2）\n
*              4. 计算比力（考虑划桨效应补偿）\n
*              5. 将比力转换到导航系并更新速度
*/
bool VelocityUpdate(const RAWDAT Rawdata_cur, const RAWDAT Rawdata_pre, InsState& InsState)
{
	double deltaT = (Rawdata_cur.time.Week - Rawdata_pre.time.Week) * 604800.0 + (Rawdata_cur.time.Second - Rawdata_pre.time.Second);

	// 1. 外推角速度、速度
	double wien[3] = { 0.0 }, wenn[3] = { 0.0 }, v[3] = { 0.0 }, gpn[3] = { 0.0 };
	Extrapolation(InsState.prev1, InsState.prev2, Rawdata_cur, wien, wenn, v, gpn);

	// 2. 求重力/哥式积分项
	double wgcor[3] = { 2 * wien[0] + wenn[0],2 * wien[1] + wenn[1],2 * wien[2] + wenn[2] };
	double wgcor_v[3] = { 0.0 };
	CrossDot(3, 3, wgcor, v, wgcor_v);
	double vgcor[3] = { (gpn[0] - wgcor_v[0]) * deltaT,(gpn[1] - wgcor_v[1]) * deltaT,(gpn[2] - wgcor_v[2]) * deltaT };

	// 3. 求比力积分项
	double Theta[3] = { (wien[0] + wenn[0]) * deltaT ,(wien[1] + wenn[1]) * deltaT,(wien[2] + wenn[2]) * deltaT };
	double Theta_[9] = { 0.0 };              // 计算旋转矢量的反对称矩阵
	SkewSymmetricMatrix(3, Theta, Theta_);
	double Theta_2[9] = { 0.0 };             // 计算反对称矩阵的1/2
	for (int i = 0; i < 9; i++) { Theta_2[i] = Theta_[i] / 2.0; }
	double I[9] = { 0.0 };                   // 定义3*3的单位阵
	EyeMatRowMajor(3, I);
	double vf_1[9] = { 0.0 };                // 计算“I-（Theta*）/2”
	MatrixSubtraction(3, 3, I, Theta_2, vf_1);

	CosineMatrix Cbn;
	Cbn.SetCbn(InsState.prev1.atti);
	double vf_2[9] = { 0.0 };                // 计算“(I-（Theta*）/2)*Cbn”
	MatrixMultiply(3, 3, 3, 3, vf_1, Cbn.DCM, vf_2);// ！！！输错矩阵大小

	//double vfb[3] = { Rawdata_cur.acc_v.X, Rawdata_cur.acc_v.Y ,Rawdata_cur.acc_v.Z };   // 省略小项
	double vk[3] = { Rawdata_cur.acc_v.X, Rawdata_cur.acc_v.Y ,Rawdata_cur.acc_v.Z };
	double vk_1[3]= { Rawdata_pre.acc_v.X, Rawdata_pre.acc_v.Y ,Rawdata_pre.acc_v.Z };
	double gk[3] = { Rawdata_cur.gyr.X, Rawdata_cur.gyr.Y ,Rawdata_cur.gyr.Z };
	double gk_1[3] = { Rawdata_pre.gyr.X, Rawdata_pre.gyr.Y ,Rawdata_pre.gyr.Z };
	double vfb1[3] = { 0.0 }, vfb2[3] = { 0.0 }, vfb3[3] = { 0.0 };
	CrossDot(3, 3, gk, vk, vfb1);
	CrossDot(3, 3, gk_1, vk, vfb2);
	CrossDot(3, 3, vk_1, gk, vfb3);
	double vfb[3] = { 0.0 };
	for (int i = 0; i < 3; i++)
	{
		vfb[i] = vk[i] + vfb1[i] / 2.0 + vfb2[i] / 12.0 + vfb3[i] / 12.0;
	}
	
	double vf[3] = { 0.0 };
	MatrixMultiply(3, 3, 3, 1, vf_2, vfb, vf);


	// 4. 计算当前速度
	InsState.cur.vel.Vn = InsState.prev1.vel.Vn + vf[0] + vgcor[0];
	InsState.cur.vel.Ve = InsState.prev1.vel.Ve + vf[1] + vgcor[1];
	InsState.cur.vel.Vd = InsState.prev1.vel.Vd + vf[2] + vgcor[2];

	return true;
}

/**
* @param[in]   Rawdata_cur  const RAWDAT&     当前历元原始数据
* @param[in]   Rawdata_pre  const RAWDAT&     前一个历元原始数据
* @param[in,out] InsState   InsState&        惯导状态（输入前一历元状态，输出当前历元状态）
* @return      bool  返回是否更新成功，true表示成功，false表示失败
* @note        位置更新函数，使用速度积分更新当前历元的位置（纬度、经度、高度）。\n
*              使用梯形积分方法，考虑地球曲率的影响。
* @par History:
*              2025/12/24, Weizhen Wang, Create
* @internals   计算步骤：\n
*              1. 计算时间间隔deltaT\n
*              2. 使用梯形积分更新高度H\n
*              3. 计算子午圈半径R_M，使用梯形积分更新纬度\n
*              4. 计算卯酉圈半径R_N（考虑线性外推），使用梯形积分更新经度
*/
bool PositionUpdate(const RAWDAT Rawdata_cur, const RAWDAT Rawdata_pre, InsState& InsState)
{
	// 1. 计算时间差
	double deltaT = (Rawdata_cur.time.Week - Rawdata_pre.time.Week) * 604800.0 + (Rawdata_cur.time.Second - Rawdata_pre.time.Second);

	// 2. 计算高程
	InsState.cur.pos.H = InsState.prev1.pos.H - (InsState.cur.vel.Vd + InsState.prev1.vel.Vd) * deltaT / 2;

	// 3. 计算纬度
	double R_M = R_WGS84 * (1 - E2_WGS84) / sqrt(pow(1 - E2_WGS84 * sin(InsState.prev1.pos.latitude) * sin(InsState.prev1.pos.latitude), 3));
	double h_average = (InsState.cur.pos.H + InsState.prev1.pos.H) / 2;
	InsState.cur.pos.latitude = InsState.prev1.pos.latitude + (InsState.cur.vel.Vn + InsState.prev1.vel.Vn) * deltaT / 2 / (R_M + h_average);

	// 4. 计算经度
	double R_N_prev1 = R_WGS84 / sqrt(1 - E2_WGS84 * sin(InsState.prev1.pos.latitude) * sin(InsState.prev1.pos.latitude));
	double R_N_prev2 = R_WGS84 / sqrt(1 - E2_WGS84 * sin(InsState.prev2.pos.latitude) * sin(InsState.prev2.pos.latitude));
	double dt_prev = (InsState.prev1.time.Week - InsState.prev2.time.Week) * 604800.0 + (InsState.prev1.time.Second - InsState.prev2.time.Second);
	double R_N = R_N_prev1 + deltaT * (R_N_prev1 - R_N_prev2) / dt_prev;
	double Lat_average = (InsState.cur.pos.latitude + InsState.prev1.pos.latitude) / 2;
	InsState.cur.pos.longitude = InsState.prev1.pos.longitude + (InsState.cur.vel.Ve + InsState.prev1.vel.Ve) * deltaT / 2 / (R_N + h_average) / cos(Lat_average);

	return true;
}