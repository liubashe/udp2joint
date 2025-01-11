#pragma once
#ifndef POSITIONH_FK_H
#define POSITIONH_FK_H

#define NODES_HAND 20

class PositionH_FK
{
public:
	PositionH_FK(double Initial_NodesPosition[NODES_HAND][3]);
	~PositionH_FK() { };

private:
	struct struct_BoneVector {
		double vector_FtHa[3];   //大拇指  //vector_FtHa = Initial_nodePosition_Ft - Initial_nodePosition_Ha;  以下同理
		double vector_Ft1Ft[3];
		double vector_Ft2Ft1[3];
		double vector_FteFt2[3];

		double vector_FiHa[3];   //食指
		double vector_Fi1Fi[3];
		double vector_Fi2Fi1[3];
		double vector_Fi3Fi2[3];
		double vector_FieFi3[3];

		double vector_FmHa[3];   //中指
		double vector_Fm1Fm[3];
		double vector_Fm2Fm1[3];
		double vector_Fm3Fm2[3];
		double vector_FmeFm3[3];

		double vector_FrHa[3];   //无名指
		double vector_Fr1Fr[3];
		double vector_Fr2Fr1[3];
		double vector_Fr3Fr2[3];
		double vector_FreFr3[3];

		double vector_FpHa[3];   //尾指
		double vector_Fp1Fp[3];
		double vector_Fp2Fp1[3];
		double vector_Fp3Fp2[3];
		double vector_FpeFp3[3];
	};
	typedef enum HANDNODESFK
	{
		HN_Hand = 0,
		HN_ThumbFinger,
		HN_ThumbFinger1,
		HN_ThumbFinger2,
		HN_IndexFinger,
		HN_IndexFinger1,
		HN_IndexFinger2,
		HN_IndexFinger3,
		HN_MiddleFinger,
		HN_MiddleFinger1,
		HN_MiddleFinger2,
		HN_MiddleFinger3,
		HN_RingFinger,
		HN_RingFinger1,
		HN_RingFinger2,
		HN_RingFinger3,
		HN_PinkyFinger,
		HN_PinkyFinger1,
		HN_PinkyFinger2,
		HN_PinkyFinger3,
	}_HandNodesFK_;
	struct_BoneVector iniVec, nowVec;
	double INITIAL_POSITION[NODES_HAND][3] = { 0 };

private:
	void MulMatrix(double matrix_3_3[9], double data_3_1[3], double data_out[3]);
	void GetCbn(double quat_nb[4], double Cbn_out[9]);
	void Copy(double(*out)[3], double(*in)[3], unsigned r);
	void Copy(double *out, double *in, unsigned r);
	void Sub(double* out, double* in1, double* in2, unsigned length);
	void Add(double* out, double* in1, double* in2, unsigned length);

public:
	void SetInitialNodesPosition(double Initial_NodesPosition[NODES_HAND][3]);

	void GetNodesPosition(double rootPosition[3], double quaternion[NODES_HAND][4], double NodesPosition_out[NODES_HAND][3]);
};

#endif // !POSITIONB_FK_H




