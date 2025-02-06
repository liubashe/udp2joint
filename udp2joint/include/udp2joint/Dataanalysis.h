#pragma once

#ifndef DATAANALYSIS_H
#define DATAANALYSIS_H


#define NODES_BODY 23
#define NODES_HAND 20
#define NODES_FACEBS_ARKIT 52
#define NODES_FACEBS_AUDIO 26

#define DC_QUAT 1e-4   //short -> double
#define DC_POSITION 1e-3   //short -> double
#define DC_POWER 1e-2  //short -> double

#include "VDMocapSDK_DataRead_DataType.h"

using namespace VDDataRead;



typedef struct STRUCT_RECEIVEDMOTIONDATA
{
	bool isUpdate = false;                               //
	int frameIndex = -1;                                 //֡���
	double batteryPower = 0;
	double frequency = 0;                                   //�豸���ݴ���Ƶ��

	int zuptResult = 0;                        //���ټ������

	double position_root[3] = { 0 };                               //���ڵ����꣬��λm

	_SensorState_ sensorState_body[NODES_BODY] = { SS_NONE };      //�豸ȫ������������ָ��������������״̬����_BodyNodes_������У���ͬ��	
	double quat_nb_body[NODES_BODY][4]/*wxyz*/ = { { 0 } };    //ȫ������������ָ�����ڵ���Ԫ��

	_SensorState_ sensorState_rHand[NODES_HAND] = { SS_NONE };     //�豸˫�֣����Ƽ���ָ��������������״̬����_HandNodes_������У���ͬ��	
	double quat_nb_rHand[NODES_HAND][4]/*wxyz*/ = { { 0 } };   //˫�֣����Ƽ���ָ�����ڵ���Ԫ��

	_SensorState_ sensorState_lHand[NODES_HAND] = { SS_NONE };     //�豸˫�֣����Ƽ���ָ��������������״̬����_HandNodes_������У���ͬ��	
	double quat_nb_lHand[NODES_HAND][4]/*wxyz*/ = { { 0 } };   //˫�֣����Ƽ���ָ�����ڵ���Ԫ��

	bool isUseFaceBlendShapesARKit = false;
	bool isUseFaceBlendShapesAudio = false;
	int faceBlendShapesARKit[NODES_FACEBS_ARKIT] = { 0 };  //���� bs ���ݣ�ͼƬ��ʽʶ��ı���
	int faceBlendShapesAudio[NODES_FACEBS_AUDIO] = { 0 };  //���� bs ���ݣ�������ʽʶ��ı���
	double localQuat_RightEyeball[4] = { 0 };  //����������
	double localQuat_LeftEyeball[4] = { 0 };   //����������

}struct_ReceivedMotionData;




// [param in] bytes udp���ն��յ�������
// [param out]calculationData ת����Ľṹ������

static bool BytestoCalculationData(unsigned char *bytes, struct_ReceivedMotionData &calculationData)
{
	//�ж������ٽ��д���,�������޸�
	//udp  bytes����Ϊ683��bytes[2] << 8 | bytes[3]����Ϊ677;
	if (static_cast<unsigned short>(bytes[2] << 8 | bytes[3]) - 3 < (NODES_BODY * 8)) { return false; }

	//isUpdate
	int offset = 7;      //����λ��ʼ���±�
	if (bytes[offset] == 1) { calculationData.isUpdate = true; }
	else { calculationData.isUpdate = false; }

	//frameIndex
	offset += 1;
	calculationData.frameIndex = bytes[offset];

	//batteryPower
	offset += 1;
	calculationData.batteryPower = bytes[offset] * DC_POWER;

	//frequency
	offset += 1;
	calculationData.frequency = bytes[offset];

	//position_root
	offset += 1;
	for (int i = 0; i < 3; i++)
	{
		calculationData.position_root[i] = (short)(bytes[offset + i * 2] << 8 | bytes[offset + i * 2 + 1]) * DC_POSITION;
	}

	//sensorState_body
	offset += 6;
	for (int i = 0; i < NODES_BODY; i++)
	{
		calculationData.sensorState_body[i] = (_SensorState_)(bytes[offset + i]);
	}

	//quaternion_body
	offset += NODES_BODY;
	for (int i = 0; i < NODES_BODY; i++)
	{
		for (int ii = 0; ii < 4; ii++)
		{
			calculationData.quat_nb_body[i][ii] = (short)(bytes[offset + i * 8 + ii * 2] << 8 | bytes[offset + i * 8 + ii * 2 + 1]) * DC_QUAT;
		}
	}

	//sensorState_rHand
	offset += NODES_BODY * 8;
	for (int i = 0; i < NODES_HAND; i++)
	{
		calculationData.sensorState_rHand[i] = (_SensorState_)(bytes[offset + i]);
	}

	//quaternion_rHand
	offset += NODES_HAND;
	for (int i = 0; i < NODES_HAND; i++)
	{
		for (int ii = 0; ii < 4; ii++)
		{
			calculationData.quat_nb_rHand[i][ii] = (short)(bytes[offset + i * 8 + ii * 2] << 8 | bytes[offset + i * 8 + ii * 2 + 1]) * DC_QUAT;
		}
	}

	//sensorState_lHand
	offset += NODES_HAND * 8;
	for (int i = 0; i < NODES_HAND; i++)
	{
		calculationData.sensorState_lHand[i] = (_SensorState_)(bytes[offset + i]);
	}

	//quaternion_lHand
	offset += NODES_HAND;
	for (int i = 0; i < NODES_HAND; i++)
	{
		for (int ii = 0; ii < 4; ii++)
		{
			calculationData.quat_nb_lHand[i][ii] = (short)(bytes[offset + i * 8 + ii * 2] << 8 | bytes[offset + i * 8 + ii * 2 + 1]) * DC_QUAT;
		}
	}

	//zuptResult
	offset += NODES_HAND * 8;
	calculationData.zuptResult = (bytes[offset]);

	//isUseFaceBlendShapesARKit
	offset += 1;
	if (bytes[offset] == 1) { calculationData.isUseFaceBlendShapesARKit = true; }
	else { calculationData.isUseFaceBlendShapesARKit = false; }

	//isUseFaceBlendShapesAudio
	offset += 1;
	if (bytes[offset] == 1) { calculationData.isUseFaceBlendShapesAudio = true; }
	else { calculationData.isUseFaceBlendShapesAudio = false; }


	//faceBlendShapes_ARKit
	offset += 1;
	for (int i = 0; i < NODES_FACEBS_ARKIT; i++)
	{
		calculationData.faceBlendShapesARKit[i] = bytes[offset + i];
	}

	//faceBlendShapes_Audio
	offset += NODES_FACEBS_ARKIT;
	for (int i = 0; i < NODES_FACEBS_AUDIO; i++)
	{
		calculationData.faceBlendShapesAudio[i] = bytes[offset + i];
	}

	//localQuat_RightEyeball
	offset += NODES_FACEBS_AUDIO;
	for (int i = 0; i < 4; i++)
	{
		calculationData.localQuat_RightEyeball[i] = (short)(bytes[offset + i * 2] << 8 | bytes[offset + i * 2 + 1]) * DC_QUAT;
	}

	//localQuat_LeftEyeball
	offset += 8;
	for (int i = 0; i < 4; i++)
	{
		calculationData.localQuat_LeftEyeball[i] = (short)(bytes[offset + i * 2] << 8 | bytes[offset + i * 2 + 1]) * DC_QUAT;
	}

	return true;
}




#endif