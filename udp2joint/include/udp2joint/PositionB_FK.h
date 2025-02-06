/****************************************************************************************
Copyright (C) 2019 VIRDYN, QX.
说明：
（1）世界坐标系(地理坐标系方向):（右手坐标系-xyz-北西天，原点是固定的（通过初始Tpose下各节点坐标来确定））
（2）模型初始姿态为Tpose，且面朝西，头顶天
（3）初始Tpose下，各节点骨骼坐标系（右手坐标系）与地理坐标系方向重合，且各节点骨骼坐标系随该骨骼移动旋转也跟着移动旋转
（4）position: 0-x-北, 1-y-西, 2-z-天, 右手坐标系，z轴指天，各节点坐标均为地理坐标系下坐标（不管是输入还是输出）
（5）quaternion: 0-qw, 1-qx, 2-qy, 3-qz
*****************************************************************************************/

#pragma once
#ifndef POSITIONB_FK_H
#define POSITIONB_FK_H

#define NODES_BODY 23

/*
* 说明：
* 世界坐标系：地理坐标系（右手坐标系-xyz-北西天，原点是固定的（通过初始Tpose下各节点坐标来确定））
* 模型初始姿态为Tpose，且面朝西，头顶天
* 初始Tpose下，各节点骨骼坐标系（右手坐标系）与地理坐标系方向重合，且各节点骨骼坐标系随该骨骼移动旋转也跟着移动旋转
* position: 0-x-北, 1-y-西, 2-z-天, 右手坐标系，z轴指天，各节点坐标均为地理坐标系下坐标（不管是输入还是输出）
* quaternion: 0-qw, 1-qx, 2-qy, 3-qz
*/
class PositionB_FK
{
public:
	PositionB_FK(double Initial_NodesPosition[NODES_BODY][3]);
	~PositionB_FK() { };


private:
	struct struct_BoneVector {
		double vector_LurHi[3];     // vector_LurHi = Initial_nodePosition_Lur - Initial_nodePosition_Hi;  以下同理
		double vector_LlrLur[3];
		double vector_FrLlr[3];
		double vector_TrFr[3];
		double vector_LulHi[3];
		double vector_LllLul[3];
		double vector_FlLll[3];
		double vector_TlFl[3];

		double vector_SpHi[3];
		double vector_Sp1Sp[3];
		double vector_Sp2Sp1[3];
		double vector_Sp3Sp2[3];
		double vector_NeSp3[3];
		double vector_HeNe[3];

		double vector_ShrSp3[3];
		double vector_AurShr[3];
		double vector_AlrAur[3];
		double vector_HarAlr[3];
		double vector_ShlSp3[3];
		double vector_AulShl[3];
		double vector_AllAul[3];
		double vector_HalAll[3];
	};
	typedef enum BODYNODESFK
	{
		BN_Hips = 0,                    ///< Hips
		BN_RightUpperLeg,               ///< Right Upper Leg
		BN_RightLowerLeg,               ///< Right Lower Leg
		BN_RightFoot,                   ///< Right Foot
		BN_RightToe,                    ///< Right Toe
		BN_LeftUpperLeg,                ///< Left Upper Leg
		BN_LeftLowerLeg,                ///< Left Lower Leg
		BN_LeftFoot,                    ///< Left Foot
		BN_LeftToe,                     ///< Left Toe
		BN_Spine,                       ///< Spine
		BN_Spine1,                      ///< Spine1
		BN_Spine2,                      ///< Spine2
		BN_Spine3,                      ///< Spine3 -- Back
		BN_Neck,                        ///< Neck
		BN_Head,                        ///< Head
		BN_RightShoulder,               ///< Right Shoulder
		BN_RightUpperArm,               ///< Right Upper Arm
		BN_RightLowerArm,               ///< Right Lower Arm
		BN_RightHand,                   ///< Right Hand
		BN_LeftShoulder,                ///< Left Shoulder
		BN_LeftUpperArm,                ///< Left Upper Arm
		BN_LeftLowerArm,                ///< Left Lower Arm
		BN_LeftHand,                    ///< Left Hand
	}_BodyNodesFK_;
	struct_BoneVector iniVec, nowVec;
	double INITIAL_POSITION[NODES_BODY][3] = { 0 };

private:
	void MulMatrix(double matrix_3_3[9], double data_3_1[3], double data_out[3]);
	void GetCbn(double quat_nb[4], double Cbn_out[9]);
	void Copy(double(*out)[3], double(*in)[3], unsigned r);
	void Copy(double *out, double *in, unsigned r);
	void Sub(double* out, double* in1, double* in2, unsigned length);
	void Add(double* out, double* in1, double* in2, unsigned length);

public:
	void SetInitialNodesPosition(double Initial_NodesPosition[NODES_BODY][3]);

	void GetNodesPosition(double rootPosition[3], double quaternion[NODES_BODY][4], double NodesPosition_out[NODES_BODY][3]);

//public:
};

#endif // !POSITIONB_FK_H




