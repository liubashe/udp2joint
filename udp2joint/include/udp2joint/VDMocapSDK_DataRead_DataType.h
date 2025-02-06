#pragma once
#ifndef VDMOCAPSDK_DATAREAD_DATATYPE_H
#define VDMOCAPSDK_DATAREAD_DATATYPE_H

#define NODES_BODY 23
#define NODES_HAND 20
#define NODES_FACEBS_ARKIT 52
#define NODES_FACEBS_AUDIO 26

namespace VDDataRead
{
	/**
	* @brief
	*   全身各节点及其序号。
	*/
	typedef enum BODYNODES
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
	}_BodyNodes_;



	/**
	* @brief
	*   Hand nodes name and its index.
	*/
	typedef enum HANDNODES
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
	}_HandNodes_;




	//Face BlendShape index
	typedef enum FACEBLENDSHAPEARKIT
	{
		ARKIT_BrowDownLeft = 0,
		ARKIT_BrowDownRight,
		ARKIT_BrowInnerUp,
		ARKIT_BrowOuterUpLeft,
		ARKIT_BrowOuterUpRight,
		ARKIT_CheekPuff,
		ARKIT_CheekSquintLeft,
		ARKIT_CheekSquintRight,
		ARKIT_EyeBlinkLeft,
		ARKIT_EyeBlinkRight,
		ARKIT_EyeLookDownLeft,
		ARKIT_EyeLookDownRight,
		ARKIT_EyeLookInLeft,
		ARKIT_EyeLookInRight,
		ARKIT_EyeLookOutLeft,
		ARKIT_EyeLookOutRight,
		ARKIT_EyeLookUpLeft,
		ARKIT_EyeLookUpRight,
		ARKIT_EyeSquintLeft,
		ARKIT_EyeSquintRight,
		ARKIT_EyeWideLeft,
		ARKIT_EyeWideRight,
		ARKIT_JawForward,
		ARKIT_JawLeft,
		ARKIT_JawOpen,
		ARKIT_JawRight,
		ARKIT_MouthClose,
		ARKIT_MouthDimpleLeft,
		ARKIT_MouthDimpleRight,
		ARKIT_MouthFrownLeft,
		ARKIT_MouthFrownRight,
		ARKIT_MouthFunnel,
		ARKIT_MouthLeft,
		ARKIT_MouthLowerDownLeft,
		ARKIT_MouthLowerDownRight,
		ARKIT_MouthPressLeft,
		ARKIT_MouthPressRight,
		ARKIT_MouthPucker,
		ARKIT_MouthRight,
		ARKIT_MouthRollLower,
		ARKIT_MouthRollUpper,
		ARKIT_MouthShrugLower,
		ARKIT_MouthShrugUpper,
		ARKIT_MouthSmileLeft,
		ARKIT_MouthSmileRight,
		ARKIT_MouthStretchLeft,
		ARKIT_MouthStretchRight,
		ARKIT_MouthUpperUpLeft,
		ARKIT_MouthUpperUpRight,
		ARKIT_NoseSneerLeft,
		ARKIT_NoseSneerRight,
		ARKIT_TongueOut,
	}_FaceBlendShapeARKit_;



	//Face BlendShape index
	typedef enum FACEBLENDSHAPEAUDIO
	{
		AUDIO_a = 0,
		AUDIO_b,
		AUDIO_c,
		AUDIO_d,
		AUDIO_e,
		AUDIO_f,
		AUDIO_g,
		AUDIO_h,
		AUDIO_i,
		AUDIO_j,
		AUDIO_k,
		AUDIO_l,
		AUDIO_m,
		AUDIO_n,
		AUDIO_o,
		AUDIO_p,
		AUDIO_q,
		AUDIO_r,
		AUDIO_s,
		AUDIO_t,
		AUDIO_u,
		AUDIO_v,
		AUDIO_w,
		AUDIO_x,
		AUDIO_y,
		AUDIO_z,
	}_FaceBlendShapeAudio_;



	typedef enum SENSORSTATE {
		SS_NONE = 0,
		SS_Well,                        //正常
		SS_NoData,                      //无数据
		SS_UnReady,                     //初始化中
		SS_BadMag,                      //磁干扰
	}_SensorState_;



	typedef enum WORLDSPACE {
		WS_Geo = 0,                        //表示世界坐标系为地理坐标系
		WS_Unity,                          //表示世界坐标系为Unity世界坐标系
		WS_UE4,                            //表示世界坐标系为UE4世界坐标系
	}_WorldSpace_;

	typedef enum GESTURE {
		GESTURE_NONE = 0,  //未知手势
		GESTURE_1,         //食指伸直，其它手指握拢（指向）
		GESTURE_2,         //剪刀手
		GESTURE_3,         //OK
		GESTURE_4,         //四
		GESTURE_5,         //掌（布）
		GESTURE_6,         //六
		GESTURE_7,         //七
		GESTURE_8,         //九
		GESTURE_9,         //大拇指与食指伸直并拢（开枪）
		GESTURE_10,        //手枪
		GESTURE_11,        //比心
		GESTURE_12,        //大拇指、食指、小指伸直，其它手指握拢（爱你）
		GESTURE_13,        //摇滚
		GESTURE_14,        //赞
		GESTURE_15,        //抓（拿）
		GESTURE_16,        //握拳（石头）
		GESTURE_17,        //手枪
		GESTURE_18,        //八
		GESTURE_19,        //踩
	}_Gesture_;



	//对于获取导入的数据，若 isUpdate = false 表示该帧未更新（及该帧是丢帧的补上的，且数据与前一帧相同）
	typedef struct MOCAPDATA
	{
		unsigned int frameIndex;                    //帧序号
		int frequency;                              //设备数据传输频率

		_SensorState_ sensorState_body[NODES_BODY] = { SS_NONE };
		double position_body[NODES_BODY][3]/*xyz-m*/ = { {0} };
		double quaternion_body[NODES_BODY][4]/*wxyz*/ = { {0} };


		_SensorState_ sensorState_rHand[NODES_HAND] = { SS_NONE };
		double position_rHand[NODES_HAND][3]/*xyz-m*/ = { {0} };
		double quaternion_rHand[NODES_HAND][4]/*wxyz*/ = { {0} };


		_SensorState_ sensorState_lHand[NODES_HAND] = { SS_NONE };
		double position_lHand[NODES_HAND][3]/*xyz-m*/ = { {0} };
		double quaternion_lHand[NODES_HAND][4]/*wxyz*/ = { {0} };

	}_MocapData_;



	//
	typedef struct VERSION
	{
		unsigned char Project_Name[26] = { 0 };
		unsigned char Author_Organization[128] = { 0 };
		unsigned char Author_Domain[26] = { 0 };
		unsigned char Author_Maintainer[26] = { 0 };
		unsigned char Version[26] = { 0 };
		unsigned char Version_Major;
		unsigned char Version_Minor;
		unsigned char Version_Patch;
	}_Version_;




}//end namespace

#endif // !VDMOCAPSDK_DATAREAD_DATATYPE_H

