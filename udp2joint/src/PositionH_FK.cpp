/****************************************************************************************
Copyright (C) 2019 VIRDYN, QX.
说明：
（1）世界坐标系(地理坐标系方向):（右手坐标系-xyz-北西天，原点是固定的（通过初始Tpose下各节点坐标来确定））
（2）模型初始姿态为Tpose，且面朝西，头顶天
（3）初始Tpose下，各节点骨骼坐标系（右手坐标系）与地理坐标系方向重合，且各节点骨骼坐标系随该骨骼移动旋转也跟着移动旋转
（4）position: 0-x-北, 1-y-西, 2-z-天, 右手坐标系，z轴指天，各节点坐标均为地理坐标系下坐标（不管是输入还是输出）
（5）quaternion: 0-qw, 1-qx, 2-qy, 3-qz
*****************************************************************************************/

#include "PositionH_FK.h"

/*
* 构造函数
*/
PositionH_FK::PositionH_FK(double Initial_NodesPosition[NODES_HAND][3])
{
	SetInitialNodesPosition(Initial_NodesPosition);
}


#pragma region sub function
/*
* 按行排列
*/
void PositionH_FK::MulMatrix(double matrix_3_3[9], double data_3_1[3], double data_out[3])
{
	for (int i = 0; i < 3; i++)
	{
		data_out[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			data_out[i] += matrix_3_3[j + i * 3] * data_3_1[j];
		}
	}
}
#pragma endregion


/*
* 初始化
* Initial_NodesPosition：模型初始状态（Tpose）时各节点的坐标，按_BodyModel_排序
*/
void PositionH_FK::SetInitialNodesPosition(double Initial_NodesPosition[NODES_HAND][3])
{
	Copy(INITIAL_POSITION, Initial_NodesPosition, NODES_HAND);

#pragma region 父子节点向量初始化
	Sub(iniVec.vector_FtHa, INITIAL_POSITION[HN_ThumbFinger], INITIAL_POSITION[HN_Hand], 3);
	Sub(iniVec.vector_Ft1Ft, INITIAL_POSITION[HN_ThumbFinger1], INITIAL_POSITION[HN_ThumbFinger], 3);
	Sub(iniVec.vector_Ft2Ft1, INITIAL_POSITION[HN_ThumbFinger2], INITIAL_POSITION[HN_ThumbFinger1], 3);


	Sub(iniVec.vector_FiHa, INITIAL_POSITION[HN_IndexFinger], INITIAL_POSITION[HN_Hand], 3);
	Sub(iniVec.vector_Fi1Fi, INITIAL_POSITION[HN_IndexFinger1], INITIAL_POSITION[HN_IndexFinger], 3);
	Sub(iniVec.vector_Fi2Fi1, INITIAL_POSITION[HN_IndexFinger2], INITIAL_POSITION[HN_IndexFinger1], 3);
	Sub(iniVec.vector_Fi3Fi2, INITIAL_POSITION[HN_IndexFinger3], INITIAL_POSITION[HN_IndexFinger2], 3);


	Sub(iniVec.vector_FmHa, INITIAL_POSITION[HN_MiddleFinger], INITIAL_POSITION[HN_Hand], 3);
	Sub(iniVec.vector_Fm1Fm, INITIAL_POSITION[HN_MiddleFinger1], INITIAL_POSITION[HN_MiddleFinger], 3);
	Sub(iniVec.vector_Fm2Fm1, INITIAL_POSITION[HN_MiddleFinger2], INITIAL_POSITION[HN_MiddleFinger1], 3);
	Sub(iniVec.vector_Fm3Fm2, INITIAL_POSITION[HN_MiddleFinger3], INITIAL_POSITION[HN_MiddleFinger2], 3);


	Sub(iniVec.vector_FrHa, INITIAL_POSITION[HN_RingFinger], INITIAL_POSITION[HN_Hand], 3);
	Sub(iniVec.vector_Fr1Fr, INITIAL_POSITION[HN_RingFinger1], INITIAL_POSITION[HN_RingFinger], 3);
	Sub(iniVec.vector_Fr2Fr1, INITIAL_POSITION[HN_RingFinger2], INITIAL_POSITION[HN_RingFinger1], 3);
	Sub(iniVec.vector_Fr3Fr2, INITIAL_POSITION[HN_RingFinger3], INITIAL_POSITION[HN_RingFinger2], 3);


	Sub(iniVec.vector_FpHa, INITIAL_POSITION[HN_PinkyFinger], INITIAL_POSITION[HN_Hand], 3);
	Sub(iniVec.vector_Fp1Fp, INITIAL_POSITION[HN_PinkyFinger1], INITIAL_POSITION[HN_PinkyFinger], 3);
	Sub(iniVec.vector_Fp2Fp1, INITIAL_POSITION[HN_PinkyFinger2], INITIAL_POSITION[HN_PinkyFinger1], 3);
	Sub(iniVec.vector_Fp3Fp2, INITIAL_POSITION[HN_PinkyFinger3], INITIAL_POSITION[HN_PinkyFinger2], 3);

#pragma endregion
}



#pragma region 不更改全局变量，只输入输出
/*
* 根据根节点坐标，更新所有其子节点坐标
*/
void PositionH_FK::GetNodesPosition(double rootPosition[3], double quaternion[NODES_HAND][4], double nodesPosition_out[NODES_HAND][3])
{


	Copy(nodesPosition_out[HN_Hand], rootPosition, 3);
	struct_BoneVector useVec;

#pragma region update vector by quaternion
	double cbn[9] = { 0 };

	//Hand -> 5 fingers root
	GetCbn(quaternion[HN_Hand], cbn);
	MulMatrix(cbn, iniVec.vector_FtHa, useVec.vector_FtHa);
	MulMatrix(cbn, iniVec.vector_FiHa, useVec.vector_FiHa);
	MulMatrix(cbn, iniVec.vector_FmHa, useVec.vector_FmHa);
	MulMatrix(cbn, iniVec.vector_FrHa, useVec.vector_FrHa);
	MulMatrix(cbn, iniVec.vector_FpHa, useVec.vector_FpHa);

	//ThumbFinger -> ThumbFingerEnd
	GetCbn(quaternion[HN_ThumbFinger], cbn);
	MulMatrix(cbn, iniVec.vector_Ft1Ft, useVec.vector_Ft1Ft);
	GetCbn(quaternion[HN_ThumbFinger1], cbn);
	MulMatrix(cbn, iniVec.vector_Ft2Ft1, useVec.vector_Ft2Ft1);
	GetCbn(quaternion[HN_ThumbFinger2], cbn);
	MulMatrix(cbn, iniVec.vector_FteFt2, useVec.vector_FteFt2);

	//IndexFinger -> IndexFingerEnd
	GetCbn(quaternion[HN_IndexFinger], cbn);
	MulMatrix(cbn, iniVec.vector_Fi1Fi, useVec.vector_Fi1Fi);
	GetCbn(quaternion[HN_IndexFinger1], cbn);
	MulMatrix(cbn, iniVec.vector_Fi2Fi1, useVec.vector_Fi2Fi1);
	GetCbn(quaternion[HN_IndexFinger2], cbn);
	MulMatrix(cbn, iniVec.vector_Fi3Fi2, useVec.vector_Fi3Fi2);
	GetCbn(quaternion[HN_IndexFinger3], cbn);
	MulMatrix(cbn, iniVec.vector_FieFi3, useVec.vector_FieFi3);

	//MiddleFinger -> MiddleFingerEnd
	GetCbn(quaternion[HN_MiddleFinger], cbn);
	MulMatrix(cbn, iniVec.vector_Fm1Fm, useVec.vector_Fm1Fm);
	GetCbn(quaternion[HN_MiddleFinger1], cbn);
	MulMatrix(cbn, iniVec.vector_Fm2Fm1, useVec.vector_Fm2Fm1);
	GetCbn(quaternion[HN_MiddleFinger2], cbn);
	MulMatrix(cbn, iniVec.vector_Fm3Fm2, useVec.vector_Fm3Fm2);
	GetCbn(quaternion[HN_MiddleFinger3], cbn);
	MulMatrix(cbn, iniVec.vector_FmeFm3, useVec.vector_FmeFm3);


	//RingFinger -> RingFingerEnd
	GetCbn(quaternion[HN_RingFinger], cbn);
	MulMatrix(cbn, iniVec.vector_Fr1Fr, useVec.vector_Fr1Fr);
	GetCbn(quaternion[HN_RingFinger1], cbn);
	MulMatrix(cbn, iniVec.vector_Fr2Fr1, useVec.vector_Fr2Fr1);
	GetCbn(quaternion[HN_RingFinger2], cbn);
	MulMatrix(cbn, iniVec.vector_Fr3Fr2, useVec.vector_Fr3Fr2);
	GetCbn(quaternion[HN_RingFinger3], cbn);
	MulMatrix(cbn, iniVec.vector_FreFr3, useVec.vector_FreFr3);

	//PinkyFinger -> PinkyFingerEnd
	GetCbn(quaternion[HN_PinkyFinger], cbn);
	MulMatrix(cbn, iniVec.vector_Fp1Fp, useVec.vector_Fp1Fp);
	GetCbn(quaternion[HN_PinkyFinger1], cbn);
	MulMatrix(cbn, iniVec.vector_Fp2Fp1, useVec.vector_Fp2Fp1);
	GetCbn(quaternion[HN_PinkyFinger2], cbn);
	MulMatrix(cbn, iniVec.vector_Fp3Fp2, useVec.vector_Fp3Fp2);
	GetCbn(quaternion[HN_PinkyFinger3], cbn);
	MulMatrix(cbn, iniVec.vector_FpeFp3, useVec.vector_FpeFp3);
#pragma endregion

#pragma region get position

	//Hand -> ThumbFingerEnd
	Add(nodesPosition_out[HN_ThumbFinger], nodesPosition_out[HN_Hand], useVec.vector_FtHa, 3);
	Add(nodesPosition_out[HN_ThumbFinger1], nodesPosition_out[HN_ThumbFinger], useVec.vector_Ft1Ft, 3);
	Add(nodesPosition_out[HN_ThumbFinger2], nodesPosition_out[HN_ThumbFinger1], useVec.vector_Ft2Ft1, 3);

	//Hand -> IndexFingerEnd
	Add(nodesPosition_out[HN_IndexFinger], nodesPosition_out[HN_Hand], useVec.vector_FiHa, 3);
	Add(nodesPosition_out[HN_IndexFinger1], nodesPosition_out[HN_IndexFinger], useVec.vector_Fi1Fi, 3);
	Add(nodesPosition_out[HN_IndexFinger2], nodesPosition_out[HN_IndexFinger1], useVec.vector_Fi2Fi1, 3);
	Add(nodesPosition_out[HN_IndexFinger3], nodesPosition_out[HN_IndexFinger2], useVec.vector_Fi3Fi2, 3);

	//Hand -> MiddleFingerEnd
	Add(nodesPosition_out[HN_MiddleFinger], nodesPosition_out[HN_Hand], useVec.vector_FmHa, 3);
	Add(nodesPosition_out[HN_MiddleFinger1], nodesPosition_out[HN_MiddleFinger], useVec.vector_Fm1Fm, 3);
	Add(nodesPosition_out[HN_MiddleFinger2], nodesPosition_out[HN_MiddleFinger1], useVec.vector_Fm2Fm1, 3);
	Add(nodesPosition_out[HN_MiddleFinger3], nodesPosition_out[HN_MiddleFinger2], useVec.vector_Fm3Fm2, 3);

	//Hand -> RingFingerEnd
	Add(nodesPosition_out[HN_RingFinger], nodesPosition_out[HN_Hand], useVec.vector_FrHa, 3);
	Add(nodesPosition_out[HN_RingFinger1], nodesPosition_out[HN_RingFinger], useVec.vector_Fr1Fr, 3);
	Add(nodesPosition_out[HN_RingFinger2], nodesPosition_out[HN_RingFinger1], useVec.vector_Fr2Fr1, 3);
	Add(nodesPosition_out[HN_RingFinger3], nodesPosition_out[HN_RingFinger2], useVec.vector_Fr3Fr2, 3);

	//Hand -> PinkyFingerEnd
	Add(nodesPosition_out[HN_PinkyFinger], nodesPosition_out[HN_Hand], useVec.vector_FpHa, 3);
	Add(nodesPosition_out[HN_PinkyFinger1], nodesPosition_out[HN_PinkyFinger], useVec.vector_Fp1Fp, 3);
	Add(nodesPosition_out[HN_PinkyFinger2], nodesPosition_out[HN_PinkyFinger1], useVec.vector_Fp2Fp1, 3);
	Add(nodesPosition_out[HN_PinkyFinger3], nodesPosition_out[HN_PinkyFinger2], useVec.vector_Fp3Fp2, 3);

#pragma endregion
}

void PositionH_FK::GetCbn(double quat_nb[4], double Cbn_out[9])
{
	double q1 = quat_nb[0];
	double q2 = quat_nb[1];
	double q3 = quat_nb[2];
	double q4 = quat_nb[3];

	Cbn_out[0] = q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4;
	Cbn_out[1] = 2 * (q2 * q3 - q1 * q4);
	Cbn_out[2] = 2 * (q2 * q4 + q1 * q3);
	Cbn_out[3] = 2 * (q2 * q3 + q1 * q4);
	Cbn_out[4] = q1 * q1 - q2 * q2 + q3 * q3 - q4 * q4;
	Cbn_out[5] = 2 * (q3 * q4 - q1 * q2);
	Cbn_out[6] = 2 * (q2 * q4 - q1 * q3);
	Cbn_out[7] = 2 * (q3 * q4 + q1 * q2);
	Cbn_out[8] = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;
}

void PositionH_FK::Copy(double(*out)[3], double(*in)[3], unsigned r)
{
	for (unsigned i = 0; i < r; i++)
	{
		for (unsigned j = 0; j < 3; j++)
		{
			out[i][j] = in[i][j];
		}
	}
}

void PositionH_FK::Copy(double *out, double *in, unsigned r)
{
	for (unsigned i = 0; i < r; i++)
	{
		out[i] = in[i];
	}
}

void PositionH_FK::Sub(double* out, double* in1, double* in2, unsigned length)
{
	for (unsigned i = 0; i < length; i++)
	{
		out[i] = in1[i] - in2[i];
	}
}

void PositionH_FK::Add(double* out, double* in1, double* in2, unsigned length)
{
	for (unsigned i = 0; i < length; i++)
	{
		out[i] = in1[i] + in2[i];
	}
}

#pragma endregion

