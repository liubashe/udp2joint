// useQTDLL_vscpp.cpp : 定义控制台应用程序的入口点。
//

#include <stdio.h>
#include <iostream>
#include "VDMocapSDK_DataRead_DataType.h"  //数据类型头文件
#include "time.h"
#include "MySocketUdp.h"
#include "PositionB_FK.h"
#include "PositionH_FK.h"
#include "Dataanalysis.h"



using namespace std;


 

void DataTransform(struct_ReceivedMotionData s_md, _MocapData_  &md)
{
	md.frameIndex = s_md.frameIndex;
	md.frequency = s_md.frequency;

	for (int i = 0; i < NODES_BODY; i++)
	{
		md.sensorState_body[i] = s_md.sensorState_body[i];
	}
	for (int i = 0; i < NODES_HAND; i++)
	{
		md.sensorState_rHand[i] = s_md.sensorState_rHand[i];
		md.sensorState_lHand[i] = s_md.sensorState_lHand[i];
	}

	for (int i = 0; i < NODES_BODY; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			md.quaternion_body[i][j] = s_md.quat_nb_body[i][j];
		}
	}


	for (int i = 0; i < NODES_HAND; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			md.quaternion_lHand[i][j] = s_md.quat_nb_lHand[i][j];
		}

		for (int j = 0; j < 4; j++)
		{
			md.quaternion_rHand[i][j] = s_md.quat_nb_rHand[i][j];
		}
	}

	for (int i = 0; i < 3; i++)
	{
		md.position_body[0][i] = s_md.position_root[i];
	}
	

}


void Copy(unsigned char *out, char *in, unsigned length)
{
	for (int i = 0; i < length; i++)
	{
		out[i] = (unsigned char)(in[i]);
	}
}


int main()
{
	//远端ip（VDMocapStudio软件所在电脑的本地ip，在VDMocapStudio软件中有显示）
	const char* dst_ip = "172.16.21.106"; //IP地址自行修改
	//远端端口
	unsigned short dst_port = 7000;

	MySocketUdp * myUdp = new MySocketUdp();

	sockaddr_in dstAddr = MySocketUdp::ToSockaddr(dst_ip, dst_port);

	//连接指令
	 unsigned char uc_ConnectsendBytes[2560] = { 0xfa, 0x00, 0x00, 0x0b, 0x04, 0x03, 0xa2, 0x53, 0x23, 0x52, 0xce, 0x32, 0x99, 0xf4, 0x32, 0xfb, 0x30 };

	//断开指令
	 unsigned char uc_DisConnectsendBytes[256] = { 0xfa, 0x00, 0x00, 0x03, 0x04, 0x0b, 0xa1, 0xfb, 0xa8 };

	 char c_recvBuffer[2560];
	 unsigned char uc_recvBuffer[2560];


	// 地理坐标系下 Tpose 骨架坐标
	double InitialPosition_body[NODES_BODY][3] = {
		{ 0,              -0.01136745,   1.01417          },
		{ 0.09608424,     -0.01918257,   0.9339905        },
		{ 0.1070137,      -0.03298733,   0.5183502        },
		{ 0.1029733,      -0.03527176,   0.09305052       },
		{ 0.1029734,       0.07543653,   0.02005178       },
		{-0.09608424,     -0.01918257,   0.9339905        },
		{-0.1070137,      -0.03298733,   0.5183502        },
		{-0.1029733,      -0.03527176,   0.09305052       },
		{-0.1029734,       0.07543653,   0.02005178       },
		{ 1.610145E-15,   -0.01136744,   1.101661         },
		{-1.967892E-16,   -0.01136744,   1.204164         },
		{ 1.722054E-16,   -0.01136744,   1.313723         },
		{ 3.313691E-17,    2.017152E-09, 1.420459         },
		{ 1.412962E-16,   -0.01136744,   1.530355         },
	    {-1.426205E-16,   -0.01136744,   1.62144          },
		{ 0.04465847,     -0.01136743,   1.45745          },
		{ 0.1871382,      -0.01136743,   1.45745          },
		{ 0.4227339,      -0.01136743,   1.45745          },
		{ 0.6813283       -0.01136743,   1.45745          },
		{-0.04465845,     -0.01136744,   1.457453         },
		{-0.1871379,      -0.01136744,   1.457453         },
		{-0.4227338,      -0.01136744,   1.457453         },
		{-0.6813283,      -0.01136744,   1.457453         },
	};
	double InitialPosition_rHand[NODES_HAND][3] = {
		{ 0.6813f,-0.0114f, 1.4575f  },
		{ 0.7121f, 0.0272f, 1.4598f  },
		{ 0.7444f, 0.0592f, 1.4597f  },
		{ 0.7669f, 0.0815f, 1.4598f  },
		{ 0.6813f,-0.0114f, 1.4575f  },
		{ 0.7857f, 0.0254f, 1.4626f  },
		{ 0.8304f, 0.0255f, 1.4604f  },
		{ 0.8557f, 0.0255f, 1.4589f  },
		{ 0.6813f,-0.0114f, 1.4575f  },
		{ 0.7871f, 0.0019f, 1.4628f  },
		{ 0.8358f, 0.0017f, 1.4595f  },
		{ 0.8663f, 0.0018f, 1.4571f  },
		{ 0.6813f,-0.0114f, 1.4575f  },
		{ 0.7802f,-0.0189f, 1.4639f  },
		{ 0.8226f,-0.0188f, 1.4599f  },
		{ 0.8520f,-0.0189f, 1.4573f  },
		{ 0.6813f,-0.0114f, 1.4575f  },
		{ 0.7716f,-0.0397f, 1.4629f  },
		{ 0.8056f,-0.0398f, 1.4611f  },
		{ 0.8270f,-0.0398f, 1.4594f  },
	};
	double InitialPosition_lHand[NODES_HAND][3] = {
		{ -0.6813f,  -0.0114f,1.4575f},
		{ -0.7121f,   0.0272f,1.4598f},
		{ -0.7444f,   0.0593f,1.4599f},
		{ -0.7666f,   0.0817f,1.4598f},
		{ -0.6813f,  -0.0114f,1.4575f},
		{ -0.7857f,   0.0254f,1.4626f},
		{ -0.8304f,   0.0253f,1.4604f},
		{ -0.8557f,   0.0254f,1.4588f},
		{ -0.6813f,  -0.0114f,1.4575f},
		{ -0.7871f,   0.0019f,1.4628f},
		{ -0.8358f,   0.0017f,1.4595f},
		{ -0.8663f,   0.0019f,1.4571f},
		{ -0.6813f,  -0.0114f,1.4575f},
		{ -0.7802f,  -0.0189f,1.4639f},
		{ -0.8226f,  -0.0189f,1.4606f},
		{ -0.8521f,  -0.0189f,1.4585f},
		{ -0.6813f,  -0.0114f,1.4575f},
		{ -0.7716f,  -0.0397f,1.4629f},
		{ -0.8056f,  -0.0396f,1.4611f},
		{ -0.8270f,  -0.0397f,1.4594f},
	};
	//================

	//计算身体各节点位置的算法
	PositionB_FK *pb_fk;
	PositionH_FK *phr_fk, *phl_fk;

	pb_fk = new PositionB_FK(InitialPosition_body);
	phr_fk = new PositionH_FK(InitialPosition_rHand);
	phl_fk = new PositionH_FK(InitialPosition_lHand);

	struct_ReceivedMotionData recvmddata;

	_MocapData_ md;

	//初始化本地udp
	myUdp->Initial(0);

	//发送连接广播命令
	myUdp->Send(dstAddr, uc_ConnectsendBytes, 17);

	int len = 0;

	//cout << endl;
	//for (int i = 0; i < 17; i++)
	//{
	//	printf("%hX  ", uc_ConnectsendBytes[i]);
	//}
	//cout << endl;


	//解析数据
	while (1)
	{
		len = myUdp->Recv(dstAddr, c_recvBuffer,2560);

		//将char数组变成unsigned char数组
		Copy(uc_recvBuffer, c_recvBuffer, 2560);

		//cout << endl;
		//for (int i = 0; i < len; i++)
		//{
		//	printf("%hX  ", uc_recvBuffer[i]);
		//}
		//cout << endl;

		//将协议进行解析变成struct_ReceivedMotionData结构体
		BytestoCalculationData(uc_recvBuffer, recvmddata);

		//将struct_ReceivedMotionData结构体的姿态和其他信息转化成_MocapData_结构体
		DataTransform(recvmddata, md);

		//根据_MocapData_的腰部节点（根节点）计算身体各个节点的位置
		pb_fk->GetNodesPosition(md.position_body[0], md.quaternion_body,md.position_body);

		//根据_MocapData_的左手掌节点，计算左手掌的各个节点的位置
		phl_fk->GetNodesPosition(md.position_body[BN_LeftHand], md.quaternion_lHand, md.position_lHand);

		//根据_MocapData_的右手掌节点，计算右手掌的各个节点的位置
		phr_fk->GetNodesPosition(md.position_body[BN_RightHand], md.quaternion_rHand, md.position_rHand);


		//这里输出的_MocapData_结构体（即结构体md）已经是完整的数据，可以用来驱动模型
		

		//打印部分数据
		cout << md.position_rHand[0][0] << "   " ;
		cout << md.position_rHand[0][1] << "   ";
		cout << md.position_rHand[0][2] << "   ";
		cout << md.position_rHand[0][3] << "   ";
		cout << endl;
	}

	//发送断开广播命令
	myUdp->Send(dstAddr, uc_DisConnectsendBytes, 9);

}



