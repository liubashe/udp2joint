/****************************************************************************************
Copyright (C) 2019 VIRDYN, QX.
说明：
（1）世界坐标系(地理坐标系方向):（右手坐标系-xyz-北西天，原点是固定的（通过初始Tpose下各节点坐标来确定））
（2）模型初始姿态为Tpose，且面朝西，头顶天
（3）初始Tpose下，各节点骨骼坐标系（右手坐标系）与地理坐标系方向重合，且各节点骨骼坐标系随该骨骼移动旋转也跟着移动旋转
（4）position: 0-x-北, 1-y-西, 2-z-天, 右手坐标系，z轴指天，各节点坐标均为地理坐标系下坐标（不管是输入还是输出）
（5）quaternion: 0-qw, 1-qx, 2-qy, 3-qz
*****************************************************************************************/

#include "PositionB_FK.h"

/*
* 构造函数
*/
PositionB_FK::PositionB_FK(double Initial_NodesPosition[NODES_BODY][3])
{
	SetInitialNodesPosition(Initial_NodesPosition);
}


/*
* 按行排列
*/
void PositionB_FK::MulMatrix(double matrix_3_3[9], double data_3_1[3], double data_out[3])
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


#pragma region main function
/*
* 初始化
* Initial_NodesPosition：模型初始状态（Tpose）时各节点的坐标，按_BodyModel_排序
*/
void PositionB_FK::SetInitialNodesPosition(double Initial_NodesPosition[NODES_BODY][3])
{
	Copy(INITIAL_POSITION, Initial_NodesPosition, NODES_BODY);


#pragma region 父子节点向量初始化
	Sub(iniVec.vector_LurHi, INITIAL_POSITION[BN_RightUpperLeg], INITIAL_POSITION[BN_Hips], 3);
	Sub(iniVec.vector_LlrLur, INITIAL_POSITION[BN_RightLowerLeg], INITIAL_POSITION[BN_RightUpperLeg], 3);
	Sub(iniVec.vector_FrLlr, INITIAL_POSITION[BN_RightFoot], INITIAL_POSITION[BN_RightLowerLeg], 3);
	Sub(iniVec.vector_TrFr, INITIAL_POSITION[BN_RightToe], INITIAL_POSITION[BN_RightFoot], 3);
	Sub(iniVec.vector_LulHi, INITIAL_POSITION[BN_LeftUpperLeg], INITIAL_POSITION[BN_Hips], 3);
	Sub(iniVec.vector_LllLul, INITIAL_POSITION[BN_LeftLowerLeg], INITIAL_POSITION[BN_LeftUpperLeg], 3);
	Sub(iniVec.vector_FlLll, INITIAL_POSITION[BN_LeftFoot], INITIAL_POSITION[BN_LeftLowerLeg], 3);
	Sub(iniVec.vector_TlFl, INITIAL_POSITION[BN_LeftToe], INITIAL_POSITION[BN_LeftFoot], 3);

	Sub(iniVec.vector_SpHi, INITIAL_POSITION[BN_Spine], INITIAL_POSITION[BN_Hips], 3);
	Sub(iniVec.vector_Sp1Sp, INITIAL_POSITION[BN_Spine1], INITIAL_POSITION[BN_Spine], 3);
	Sub(iniVec.vector_Sp2Sp1, INITIAL_POSITION[BN_Spine2], INITIAL_POSITION[BN_Spine1], 3);
	Sub(iniVec.vector_Sp3Sp2, INITIAL_POSITION[BN_Spine3], INITIAL_POSITION[BN_Spine2], 3);
	Sub(iniVec.vector_NeSp3, INITIAL_POSITION[BN_Neck], INITIAL_POSITION[BN_Spine3], 3);
	Sub(iniVec.vector_HeNe, INITIAL_POSITION[BN_Head], INITIAL_POSITION[BN_Neck], 3);

	Sub(iniVec.vector_ShrSp3, INITIAL_POSITION[BN_RightShoulder], INITIAL_POSITION[BN_Spine3], 3);
	Sub(iniVec.vector_AurShr, INITIAL_POSITION[BN_RightUpperArm], INITIAL_POSITION[BN_RightShoulder], 3);
	Sub(iniVec.vector_AlrAur, INITIAL_POSITION[BN_RightLowerArm], INITIAL_POSITION[BN_RightUpperArm], 3);
	Sub(iniVec.vector_HarAlr, INITIAL_POSITION[BN_RightHand], INITIAL_POSITION[BN_RightLowerArm], 3);
	Sub(iniVec.vector_ShlSp3, INITIAL_POSITION[BN_LeftShoulder], INITIAL_POSITION[BN_Spine3], 3);
	Sub(iniVec.vector_AulShl, INITIAL_POSITION[BN_LeftUpperArm], INITIAL_POSITION[BN_LeftShoulder], 3);
	Sub(iniVec.vector_AllAul, INITIAL_POSITION[BN_LeftLowerArm], INITIAL_POSITION[BN_LeftUpperArm], 3);
	Sub(iniVec.vector_HalAll, INITIAL_POSITION[BN_LeftHand], INITIAL_POSITION[BN_LeftLowerArm], 3);
#pragma endregion

}



void PositionB_FK::GetNodesPosition(double rootPosition[3], double quaternion[NODES_BODY][4], double nodesPosition_out[NODES_BODY][3])
{
	Copy(nodesPosition_out[BN_Hips], rootPosition, 3);
	struct_BoneVector useVec;

#pragma region update vector by quaternion
	double cbn[9] = { 0 };

	//Hips -> Spine0, RightToe, LeftToe
	GetCbn(quaternion[BN_Hips], cbn);
	MulMatrix(cbn, iniVec.vector_LurHi, useVec.vector_LurHi);
	MulMatrix(cbn, iniVec.vector_LulHi, useVec.vector_LulHi);
	MulMatrix(cbn, iniVec.vector_SpHi, useVec.vector_SpHi);
	GetCbn(quaternion[BN_RightUpperLeg], cbn);
	MulMatrix(cbn, iniVec.vector_LlrLur, useVec.vector_LlrLur);
	GetCbn(quaternion[BN_RightLowerLeg], cbn);
	MulMatrix(cbn, iniVec.vector_FrLlr, useVec.vector_FrLlr);
	GetCbn(quaternion[BN_RightFoot], cbn);
	MulMatrix(cbn, iniVec.vector_TrFr, useVec.vector_TrFr);
	GetCbn(quaternion[BN_LeftUpperLeg], cbn);
	MulMatrix(cbn, iniVec.vector_LllLul, useVec.vector_LllLul);
	GetCbn(quaternion[BN_LeftLowerLeg], cbn);
	MulMatrix(cbn, iniVec.vector_FlLll, useVec.vector_FlLll);
	GetCbn(quaternion[BN_LeftFoot], cbn);
	MulMatrix(cbn, iniVec.vector_TlFl, useVec.vector_TlFl);

	//Spine0 -> Head, RightHand, LeftHand
	GetCbn(quaternion[BN_Spine], cbn);
	MulMatrix(cbn, iniVec.vector_Sp1Sp, useVec.vector_Sp1Sp);
	GetCbn(quaternion[BN_Spine1], cbn);
	MulMatrix(cbn, iniVec.vector_Sp2Sp1, useVec.vector_Sp2Sp1);
	GetCbn(quaternion[BN_Spine2], cbn);
	MulMatrix(cbn, iniVec.vector_Sp3Sp2, useVec.vector_Sp3Sp2);
	GetCbn(quaternion[BN_Spine3], cbn);
	MulMatrix(cbn, iniVec.vector_NeSp3, useVec.vector_NeSp3);
	MulMatrix(cbn, iniVec.vector_ShrSp3, useVec.vector_ShrSp3);
	MulMatrix(cbn, iniVec.vector_ShlSp3, useVec.vector_ShlSp3);
	GetCbn(quaternion[BN_Neck], cbn);
	MulMatrix(cbn, iniVec.vector_HeNe, useVec.vector_HeNe);
	GetCbn(quaternion[BN_RightShoulder], cbn);
	MulMatrix(cbn, iniVec.vector_AurShr, useVec.vector_AurShr);
	GetCbn(quaternion[BN_RightUpperArm], cbn);
	MulMatrix(cbn, iniVec.vector_AlrAur, useVec.vector_AlrAur);
	GetCbn(quaternion[BN_RightLowerArm], cbn);
	MulMatrix(cbn, iniVec.vector_HarAlr, useVec.vector_HarAlr);
	GetCbn(quaternion[BN_LeftShoulder], cbn);
	MulMatrix(cbn, iniVec.vector_AulShl, useVec.vector_AulShl);
	GetCbn(quaternion[BN_LeftUpperArm], cbn);
	MulMatrix(cbn, iniVec.vector_AllAul, useVec.vector_AllAul);
	GetCbn(quaternion[BN_LeftLowerArm], cbn);
	MulMatrix(cbn, iniVec.vector_HalAll, useVec.vector_HalAll);

#pragma endregion

#pragma region get position
	//Hips -> RightFoot
	Add(nodesPosition_out[BN_RightUpperLeg], nodesPosition_out[BN_Hips], useVec.vector_LurHi, 3);
	Add(nodesPosition_out[BN_RightLowerLeg], nodesPosition_out[BN_RightUpperLeg], useVec.vector_LlrLur, 3);
	Add(nodesPosition_out[BN_RightFoot], nodesPosition_out[BN_RightLowerLeg], useVec.vector_FrLlr, 3);
	Add(nodesPosition_out[BN_RightToe], nodesPosition_out[BN_RightFoot], useVec.vector_TrFr, 3);

	//Hips -> LeftFoot
	Add(nodesPosition_out[BN_LeftUpperLeg], nodesPosition_out[BN_Hips], useVec.vector_LulHi, 3);
	Add(nodesPosition_out[BN_LeftLowerLeg], nodesPosition_out[BN_LeftUpperLeg], useVec.vector_LllLul, 3);
	Add(nodesPosition_out[BN_LeftFoot], nodesPosition_out[BN_LeftLowerLeg], useVec.vector_FlLll, 3);
	Add(nodesPosition_out[BN_LeftToe], nodesPosition_out[BN_LeftFoot], useVec.vector_TlFl, 3);

	//Hips -> Spine3
	Add(nodesPosition_out[BN_Spine], nodesPosition_out[BN_Hips], useVec.vector_SpHi, 3);
	Add(nodesPosition_out[BN_Spine1], nodesPosition_out[BN_Spine], useVec.vector_Sp1Sp, 3);
	Add(nodesPosition_out[BN_Spine2], nodesPosition_out[BN_Spine1], useVec.vector_Sp2Sp1, 3);
	Add(nodesPosition_out[BN_Spine3], nodesPosition_out[BN_Spine2], useVec.vector_Sp3Sp2, 3);

	//Spine3 -> Head
	Add(nodesPosition_out[BN_Neck], nodesPosition_out[BN_Spine3], useVec.vector_NeSp3, 3);
	Add(nodesPosition_out[BN_Head], nodesPosition_out[BN_Neck], useVec.vector_HeNe, 3);

	//Spine3 -> RightHand
	Add(nodesPosition_out[BN_RightShoulder], nodesPosition_out[BN_Spine3], useVec.vector_ShrSp3, 3);
	Add(nodesPosition_out[BN_RightUpperArm], nodesPosition_out[BN_RightShoulder], useVec.vector_AurShr, 3);
	Add(nodesPosition_out[BN_RightLowerArm], nodesPosition_out[BN_RightUpperArm], useVec.vector_AlrAur, 3);
	Add(nodesPosition_out[BN_RightHand], nodesPosition_out[BN_RightLowerArm], useVec.vector_HarAlr, 3);

	//Spine3 -> LeftHand
	Add(nodesPosition_out[BN_LeftShoulder], nodesPosition_out[BN_Spine3], useVec.vector_ShlSp3, 3);
	Add(nodesPosition_out[BN_LeftUpperArm], nodesPosition_out[BN_LeftShoulder], useVec.vector_AulShl, 3);
	Add(nodesPosition_out[BN_LeftLowerArm], nodesPosition_out[BN_LeftUpperArm], useVec.vector_AllAul, 3);
	Add(nodesPosition_out[BN_LeftHand], nodesPosition_out[BN_LeftLowerArm], useVec.vector_HalAll, 3);


#pragma endregion
}



void PositionB_FK::GetCbn(double quat_nb[4], double Cbn_out[9])
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

void PositionB_FK::Copy(double(*out)[3], double(*in)[3], unsigned r)
{
	for (unsigned i = 0; i < r; i++)
	{
		for (unsigned j = 0; j < 3; j++)
		{
			out[i][j] = in[i][j];
		}
	}
}


void PositionB_FK::Copy(double *out, double *in, unsigned r)
{
	for (unsigned i = 0; i < r; i++)
	{
		out[i] = in[i];
	}
}


void PositionB_FK::Sub(double* out, double* in1, double* in2, unsigned length)
{
	for (unsigned i = 0; i < length; i++)
	{
		out[i] = in1[i] - in2[i];
	}
}

void PositionB_FK::Add(double* out, double* in1, double* in2, unsigned length)
{
	for (unsigned i = 0; i < length; i++)
	{
		out[i] = in1[i] + in2[i];
	}
}

