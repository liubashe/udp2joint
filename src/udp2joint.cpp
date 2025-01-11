#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <iostream>
#include "VDMocapSDK_DataRead_DataType.h"  //数据类型头文件
#include "time.h"
#include "MySocketUdp.h"
#include "PositionB_FK.h"
#include "PositionH_FK.h"
#include "Dataanalysis.h"

using namespace std;

void Copy(unsigned char *out, char *in, unsigned length){
	for (int i = 0; i < length; i++){
		out[i] = (unsigned char)(in[i]);
	}
}

struct Quaternion {
    double w, x, y, z;
};
// 计算四元数的共轭
Quaternion quaternion_conjugate(const Quaternion& q) {
    return {q.w, -q.x, -q.y, -q.z};
}
// 计算两个四元数的乘积
Quaternion quaternion_multiply(const Quaternion& q1, const Quaternion& q2) {
    return {
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    };
}
// 计算相对四元数
Quaternion quaternion_relative(const Quaternion& q1, const Quaternion& q2) {
    Quaternion q1_conj = quaternion_conjugate(q1);
    return quaternion_multiply(q1_conj, q2);
}
// 四元数转换为欧拉角
std::vector<double> quaternion_to_euler(const Quaternion& q) {
    double roll = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    double pitch = asin(2 * (q.w * q.y - q.z * q.x));
    double yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    return {roll, pitch, yaw};
}
// 计算四元数的模长
double quaternion_norm(const Quaternion& q) {
    return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// 计算两个四元数之间的空间夹角
double quaternion_angle(const Quaternion& q1, const Quaternion& q2) {
    Quaternion q_rel = quaternion_relative(q1, q2);
    double angle = 2 * std::acos(q_rel.w / quaternion_norm(q_rel));
    return angle;
}
// 计算两个四元数之间的旋转轴
std::vector<double> quaternion_axis(const Quaternion& q1, const Quaternion& q2) {
    Quaternion q_rel = quaternion_relative(q1, q2);
    double norm = std::sqrt(q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z);
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "udp2joint");
    ros::NodeHandle nh;
    // 创建发布者
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 20);
    // 设置发布频率
    ros::Rate loop_rate(60);  // 10 Hz
    // 添加关节名称
    std::vector<string> joint_name_;
    joint_name_ = {
        "Left_Arm_Joint1","Left_Arm_Joint2","Left_Arm_Joint3","Left_Arm_Joint4","Left_Arm_Joint5","Left_Arm_Joint6","Left_Arm_Joint7",
        "Right_Arm_Joint1","Right_Arm_Joint2","Right_Arm_Joint3","Right_Arm_Joint4","Right_Arm_Joint5","Right_Arm_Joint6","Right_Arm_Joint7",
        "Body_Joint1","Body_Joint2","Body_Joint3","Body_Joint4","Neck_Joint1","Neck_Joint2"
    };
    // 设置关节初始位置
    std::vector<double> joint_init_angle={0,0.3746,0.5241,0.1204,-0.5028,-0.5396,-0.2679,
                                        0.4583,-0.3746,-0.5241,-0.1204,0.5027,0.5396,0.2679};
    std::vector<double> joint_position_;
    joint_position_.resize(20,0.0);
    for(int i=0;i<14;i++)
    {
        joint_position_[i]=joint_init_angle[i];

    }
    // std::map<std::string, double> initial_positions;
    // // 获取initial_positions参数
    // if (ros::param::get("initial_positions", initial_positions)) {
    //     std::cout << "Initial Positions: \n";
    //     int num=0;
    //     for (const auto& pair : initial_positions) {
    //         joint_init_angle[num]=(double)pair.second;
    //         ROS_INFO("%s: %f", pair.first.c_str(), pair.second);
    //         num++;
    //     }
    //     std::cout << std::endl;
    // } 
    // else {
    //     ROS_ERROR("Failed to get joint_init_angle from parameter server.");
    //     return 1;
    // }

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
    //转存数组
	char c_recvBuffer[2560];
	unsigned char uc_recvBuffer[2560];
    //四元数数据
    struct_ReceivedMotionData recvmddata;

    //初始化本地udp
	myUdp->Initial(0);
	//发送连接广播命令
	myUdp->Send(dstAddr, uc_ConnectsendBytes, 17);

	int len = 0;

    while (ros::ok()) {
        //接收UDP数据
        len = myUdp->Recv(dstAddr, c_recvBuffer,2560);
		//将char数组变成unsigned char数组
		Copy(uc_recvBuffer, c_recvBuffer, 2560);
		//将协议进行解析变成struct_ReceivedMotionData结构体
		BytestoCalculationData(uc_recvBuffer, recvmddata);

        // 更新关节位置

        //根据四元数计算关节角度
        cout << recvmddata.quat_nb_body[19] << endl;//左肩
        cout << recvmddata.quat_nb_body[20] << endl;//左大臂
        //左臂
        Quaternion qls = {recvmddata.quat_nb_body[19][0],recvmddata.quat_nb_body[19][1],recvmddata.quat_nb_body[19][2],recvmddata.quat_nb_body[19][3]};
        Quaternion qlu = {recvmddata.quat_nb_body[20][0],recvmddata.quat_nb_body[20][1],recvmddata.quat_nb_body[20][2],recvmddata.quat_nb_body[20][3]};
        Quaternion qld = {recvmddata.quat_nb_body[21][0],recvmddata.quat_nb_body[21][1],recvmddata.quat_nb_body[21][2],recvmddata.quat_nb_body[21][3]};
        Quaternion qlh = {recvmddata.quat_nb_body[22][0],recvmddata.quat_nb_body[22][1],recvmddata.quat_nb_body[22][2],recvmddata.quat_nb_body[22][3]};
        // 左大臂
        Quaternion q_lsu = quaternion_relative(qls, qlu);
        std::vector<double> euler_angles = quaternion_to_euler(q_lsu);
        joint_position_[0]= (0.8-euler_angles[1])/2*3.1415926;
        joint_position_[1]= (-euler_angles[2])/2*3.1415926;
        joint_position_[2]= (-0.8-euler_angles[0])/2*3.1415926;
        // 左小臂
        double q_ud = quaternion_angle(qlu, qld);
        Quaternion q_lud = quaternion_relative(qlu, qld);
        std::vector<double> ud_angles = quaternion_to_euler(q_lud);
        joint_position_[3]= (0.8+ud_angles[2])/2*3.1415926;
        //左手
        Quaternion q_ldh = quaternion_relative(qld, qlh);
        std::vector<double> h_angles = quaternion_to_euler(q_ldh);
        joint_position_[4]= (0.8-h_angles[1])/2*3.1415926;
        joint_position_[5]= (h_angles[2])/2*3.1415926;
        joint_position_[6]= (0.8-h_angles[0])/2*3.1415926;

        //右臂
        Quaternion qrs = {recvmddata.quat_nb_body[15][0],recvmddata.quat_nb_body[15][1],recvmddata.quat_nb_body[15][2],recvmddata.quat_nb_body[15][3]};
        Quaternion qru = {recvmddata.quat_nb_body[16][0],recvmddata.quat_nb_body[16][1],recvmddata.quat_nb_body[16][2],recvmddata.quat_nb_body[16][3]};
        Quaternion qrd = {recvmddata.quat_nb_body[17][0],recvmddata.quat_nb_body[17][1],recvmddata.quat_nb_body[17][2],recvmddata.quat_nb_body[17][3]};
        Quaternion qrh = {recvmddata.quat_nb_body[18][0],recvmddata.quat_nb_body[18][1],recvmddata.quat_nb_body[18][2],recvmddata.quat_nb_body[18][3]};
        // 右大臂
        Quaternion q_rsu = quaternion_relative(qrs, qru);
        std::vector<double> rsu_angles = quaternion_to_euler(q_rsu);
        joint_position_[7]= (-0.8-rsu_angles[1])/2*3.1415926;
        joint_position_[8]= (-rsu_angles[2])/2*3.1415926;
        joint_position_[9]= (-0.8+rsu_angles[0])/2*3.1415926;
        // 右小臂
        Quaternion q_rud = quaternion_relative(qru, qrd);
        std::vector<double> rud_angles = quaternion_to_euler(q_rud);
        joint_position_[10]= (-rud_angles[2])/2*3.1415926;
        //右手
        Quaternion q_rdh = quaternion_relative(qrd, qrh);
        std::vector<double> rdh_angles = quaternion_to_euler(q_rdh);
        joint_position_[11]= (0.8-rdh_angles[1])/2*3.1415926;
        joint_position_[12]= (rdh_angles[2])/2*3.1415926;
        joint_position_[13]= (0.8-rdh_angles[0])/2*3.1415926;

        //头
        Quaternion qn = {recvmddata.quat_nb_body[13][0],recvmddata.quat_nb_body[13][1],recvmddata.quat_nb_body[13][2],recvmddata.quat_nb_body[13][3]};
        Quaternion qh = {recvmddata.quat_nb_body[14][0],recvmddata.quat_nb_body[14][1],recvmddata.quat_nb_body[14][2],recvmddata.quat_nb_body[14][3]};
        Quaternion q_nh = quaternion_relative(qn, qh);
        std::vector<double> head_angles = quaternion_to_euler(q_nh);
        joint_position_[18]= (head_angles[2])/2*3.1415926;
        joint_position_[19]= (head_angles[0])/2*3.1415926;

        // 创建JointState消息
        sensor_msgs::JointState joint_state;
        // 设置时间戳
        joint_state.header.stamp = ros::Time::now();
        // 设置关节名称
        joint_state.name = joint_name_;
        // 设置关节位置
        joint_state.position = joint_position_;
        // 发布关节状态
        joint_state_pub.publish(joint_state);
        // 等待下一个周期
        loop_rate.sleep();
    }
    //发送断开广播命令
	myUdp->Send(dstAddr, uc_DisConnectsendBytes, 9);
    return 0;
}