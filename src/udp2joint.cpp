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

#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>

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
std::vector<double> quaternion_to_euler_xyz(const Quaternion& q) {
    double roll = atan2(2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y));
    double pitch = asin(2 * (q.w * q.y - q.z * q.x));
    double yaw = atan2(2 * (q.x * q.z + q.w * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    return {roll, pitch, yaw};
}
std::vector<double> quaternion_to_euler_xzx(const Quaternion& q) {
    // 第一个旋转 alpha (绕 X 轴)
    double alpha = atan2(2 * (q.x * q.z + q.w * q.y), 2 * (q.w * q.x - q.y * q.z));
    // 第二个旋转 beta (绕 Z 轴)
    double beta = acos(1 - 2 * (q.x * q.x + q.z * q.z));
    // 第三个旋转 gamma (绕 X 轴)
    double gamma = atan2(2 * (q.x * q.z - q.w * q.y), 2 * (q.w * q.x + q.y * q.z));
    return {alpha, beta, gamma};
}
std::vector<double> quaternion_to_euler_xyx(const Quaternion& q) {
    double roll1 = atan2(2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y));
    double pitch = asin(2 * (q.w * q.y - q.z * q.x));
    double roll2 = atan2(2 * (q.x * q.z + q.w * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    return {roll1, pitch, roll2};
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
void quaternion_axis(const Quaternion& q1, const Quaternion& q2) {
    Quaternion q_rel = quaternion_relative(q1, q2);
    double norm = std::sqrt(q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z);
}
// 四元数向量归一化
std::vector<double> quaternion_normalize(const Quaternion& q) {
    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.x / norm, q.y / norm, q.z / norm};
}
// 连杆方式求解球形关节角度
std::vector<double> quaternion_to_axis_angle(const Quaternion& q, const Quaternion& p0_q) {
    double angle = 2 * acos(q.w);  // 总旋转角度
    std::vector<double> q_norm = quaternion_normalize(q);
    std::vector<double> p_norm = quaternion_normalize(p0_q);
    double projection = (q_norm[0] * p_norm[0] + q_norm[1] * p_norm[1] + q_norm[2] * p_norm[2]);
    double spin_angle = angle * projection;//杆方向上的旋转角度
    
    Quaternion q_conj = quaternion_conjugate(q);
    Quaternion p_q = quaternion_multiply(quaternion_multiply(q, p0_q), q_conj);//q*q0*q-1
    
    double ang_1 = atan2(p_q.y, p_q.z);
    double ang_2 = atan2(p_q.x, p_q.z);
    double ang_4 = atan2(p_q.y, p_q.x);
    double ang_5 = atan2(p_q.z, p_q.y);
    double ang_6 = atan2(sqrt(p_q.y*p_q.y+p_q.z*p_q.z), p_q.x);


    return {ang_1, ang_2, spin_angle, ang_4, ang_5, ang_6, spin_angle};  // 返回旋转轴 (x, y, z) 和旋转角度
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "udp2joint");
    ros::NodeHandle nh;
    // 创建发布者
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_command", 20);
    // 设置发布频率
    ros::Rate loop_rate(60);  // 10 Hz
    
    //远端ip（VDMocapStudio软件所在电脑的本地ip，在VDMocapStudio软件中有显示）
	const char* dst_ip = "172.16.21.93"; //IP地址自行修改
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

    // 加载机器人模型（URDF + SRDF）
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    // 创建 PlanningScene 对象
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in")
                                                        << " self collision");
    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    // 创建 RobotState 对象
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    robot_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("body_neck_arms");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    // 获取初始位置
    std::vector<double> joint_position_;
    robot_state->copyJointGroupPositions(joint_model_group, joint_position_);


    while (ros::ok()) {
        //接收UDP数据
        len = myUdp->Recv(dstAddr, c_recvBuffer,2560);
		//将char数组变成unsigned char数组
		Copy(uc_recvBuffer, c_recvBuffer, 2560);
		//将协议进行解析变成struct_ReceivedMotionData结构体
		BytestoCalculationData(uc_recvBuffer, recvmddata);

        // 更新关节位置
        //左臂
        Quaternion qls = {recvmddata.quat_nb_body[19][0],recvmddata.quat_nb_body[19][1],recvmddata.quat_nb_body[19][2],recvmddata.quat_nb_body[19][3]};
        Quaternion qlu = {recvmddata.quat_nb_body[20][0],recvmddata.quat_nb_body[20][1],recvmddata.quat_nb_body[20][2],recvmddata.quat_nb_body[20][3]};
        Quaternion qld = {recvmddata.quat_nb_body[21][0],recvmddata.quat_nb_body[21][1],recvmddata.quat_nb_body[21][2],recvmddata.quat_nb_body[21][3]};
        Quaternion qlh = {recvmddata.quat_nb_body[22][0],recvmddata.quat_nb_body[22][1],recvmddata.quat_nb_body[22][2],recvmddata.quat_nb_body[22][3]};
        Quaternion ql = {0, -1, 0, 0};//左侧虚拟杆初始坐标
        // 左大臂
        Quaternion q_lsu = quaternion_relative(qls, qlu);
        std::vector<double> lsu_angles = quaternion_to_axis_angle(q_lsu, ql);
        if(lsu_angles[0] > 0) lsu_angles[0] -= 3.14;
        else lsu_angles[0] += 3.14;
        joint_position_[4]= (lsu_angles[0]);
        if(lsu_angles[1] > 0) lsu_angles[1] -= 3.14;
        else lsu_angles[1] += 3.14;
        joint_position_[5]= (lsu_angles[1]);
        joint_position_[6]= (lsu_angles[2]);
        // 左小臂
        Quaternion q_lud = quaternion_relative(qlu, qld);
        std::vector<double> lud_angles = quaternion_to_axis_angle(q_lud, ql);
        joint_position_[7]= (-2.2+lud_angles[3]);
        if(joint_position_[7] < -3) joint_position_[7] = 0.6;
        //左手
        Quaternion q_ldh = quaternion_relative(qld, qlh);
        std::vector<double> ldh_angles = quaternion_to_axis_angle(q_ldh, ql);
        joint_position_[8]= (ldh_angles[4]);
        joint_position_[9]= (2.2-ldh_angles[5]);
        joint_position_[10]= (ldh_angles[6]);

        //右臂
        Quaternion qrs = {recvmddata.quat_nb_body[15][0],recvmddata.quat_nb_body[15][1],recvmddata.quat_nb_body[15][2],recvmddata.quat_nb_body[15][3]};
        Quaternion qru = {recvmddata.quat_nb_body[16][0],recvmddata.quat_nb_body[16][1],recvmddata.quat_nb_body[16][2],recvmddata.quat_nb_body[16][3]};
        Quaternion qrd = {recvmddata.quat_nb_body[17][0],recvmddata.quat_nb_body[17][1],recvmddata.quat_nb_body[17][2],recvmddata.quat_nb_body[17][3]};
        Quaternion qrh = {recvmddata.quat_nb_body[18][0],recvmddata.quat_nb_body[18][1],recvmddata.quat_nb_body[18][2],recvmddata.quat_nb_body[18][3]};
        Quaternion qr = {0, 1, 0, 0};
        // 右大臂
        Quaternion q_rsu = quaternion_relative(qrs, qru);
        std::vector<double> rsu_angles = quaternion_to_axis_angle(q_rsu, qr);
        if(rsu_angles[0] > 0) rsu_angles[0] -= 3.14;
        else rsu_angles[0] += 3.14;
        joint_position_[13]= (-rsu_angles[0]);
        if(rsu_angles[1] > 0) rsu_angles[1] -= 3.14;
        else rsu_angles[1] += 3.14;
        joint_position_[14]= (rsu_angles[1]);
        joint_position_[15]= (-0.7+rsu_angles[2]);
        // 右小臂
        Quaternion q_rud = quaternion_relative(qru, qrd);
        std::vector<double> rud_angles = quaternion_to_axis_angle(q_rud, qr);
        joint_position_[16]= (-0.7+rud_angles[3]);
        //右手
        Quaternion q_rdh = quaternion_relative(qrd, qrh);
        std::vector<double> rdh_angles = quaternion_to_axis_angle(q_rdh, qr);
        joint_position_[17]= (-0.5+rdh_angles[4]);
        joint_position_[18]= (rdh_angles[5]);
        joint_position_[19]= (rdh_angles[6]);

        //头
        Quaternion qn = {recvmddata.quat_nb_body[13][0],recvmddata.quat_nb_body[13][1],recvmddata.quat_nb_body[13][2],recvmddata.quat_nb_body[13][3]};
        Quaternion qh = {recvmddata.quat_nb_body[14][0],recvmddata.quat_nb_body[14][1],recvmddata.quat_nb_body[14][2],recvmddata.quat_nb_body[14][3]};
        Quaternion q_nh = quaternion_relative(qn, qh);
        std::vector<double> head_angles = quaternion_to_euler(q_nh);
        joint_position_[11]= (head_angles[2]);
        joint_position_[12]= (-head_angles[0]);


        // 设置关节角度
        robot_state->setJointGroupPositions(joint_model_group, joint_position_);

        // ROS_INFO_STREAM ( "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));
        // ROS_INFO("Joint %s: %f", joint_names[1].c_str(), joint_position_[1]);
        // 关节限制
        robot_state->enforceBounds(); 
        // ROS_INFO_STREAM("Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));
        // ROS_INFO("Joint %s: %f", joint_names[1].c_str(), joint_position_[1]);
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        // ROS_INFO("Joint %s: %f", joint_names[1].c_str(), joint_values[1]);
        for (std::size_t i = 13; i < 19; ++i){
            ROS_INFO("Joint %s: %f->%f", joint_names[i].c_str(), joint_position_[i], joint_values[i]);
        }
        

        // 检查自碰撞
        current_state.setJointGroupPositions(joint_model_group, joint_values);
        collision_result.clear();
        planning_scene.checkSelfCollision(collision_request, collision_result);

        // if (collision_result.collision)
        if(0)
        {
            ROS_WARN("Self-collision detected!");
            // 发生碰撞时，阻止执行操作
            // 可以在这里添加相应的处理逻辑，例如停止机器人或调整关节角度等
        }
        // else{
        //     ROS_INFO("No collision detected.");
        // }
        else{
            // 创建JointState消息
            sensor_msgs::JointState joint_state;
            // 设置时间戳
            joint_state.header.stamp = ros::Time::now();
            // 设置关节名称
            joint_state.name = joint_names;
            // 设置关节位置
            joint_state.position = joint_values;
            // 发布关节状态
            joint_state_pub.publish(joint_state);
        }
        
        // 等待下一个周期
        loop_rate.sleep();
    }
    //发送断开广播命令
	myUdp->Send(dstAddr, uc_DisConnectsendBytes, 9);
    return 0;
}