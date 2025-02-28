#include <stdio.h>
#include <iostream>
#include <thread>
#include <vector>
#include <std_msgs/Int32.h>
#include <queue>

#include "VDMocapSDK_DataRead_DataType.h"  //数据类型头文件
#include "time.h"
#include "MySocketUdp.h"
#include "PositionB_FK.h"
#include "PositionH_FK.h"
#include "Dataanalysis.h"
#include "qua_turn.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;

int command_model = 1;
void modelCallback(const std_msgs::Int32::ConstPtr& msg){
    command_model = msg->data;
    ROS_INFO("model: %d", command_model);
}

void Copy(unsigned char *out, char *in, unsigned length){
	for (int i = 0; i < length; i++){
		out[i] = (unsigned char)(in[i]);
	}
}

void receive(std::vector<double> &joint_position_){

    //远端ip（VDMocapStudio软件所在电脑的本地ip，在VDMocapStudio软件中有显示）
	const char* dst_ip = "192.168.26.103"; //IP地址自行修改
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

    while(1){
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
        // //左手
        // Quaternion q_ldh = quaternion_relative(qld, qlh);
        // std::vector<double> ldh_angles = quaternion_to_axis_angle(q_ldh, ql);
        // joint_position_[8]= (ldh_angles[4]);
        // joint_position_[9]= (-ldh_angles[5]);
        // joint_position_[10]= (ldh_angles[6]);

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
        // //右手
        // Quaternion q_rdh = quaternion_relative(qrd, qrh);
        // std::vector<double> rdh_angles = quaternion_to_axis_angle(q_rdh, qr);
        // joint_position_[17]= (-0.5+rdh_angles[4]);
        // joint_position_[18]= (rdh_angles[5]);
        // joint_position_[19]= (rdh_angles[6]);

        // //头
        // Quaternion qn = {recvmddata.quat_nb_body[13][0],recvmddata.quat_nb_body[13][1],recvmddata.quat_nb_body[13][2],recvmddata.quat_nb_body[13][3]};
        // Quaternion qh = {recvmddata.quat_nb_body[14][0],recvmddata.quat_nb_body[14][1],recvmddata.quat_nb_body[14][2],recvmddata.quat_nb_body[14][3]};
        // Quaternion q_nh = quaternion_relative(qn, qh);
        // std::vector<double> head_angles = quaternion_to_euler(q_nh);
        // joint_position_[11]= (head_angles[2]);
        // joint_position_[12]= (-head_angles[0]);

    }
    //发送断开广播命令
	myUdp->Send(dstAddr, uc_DisConnectsendBytes, 9);
}

void sendCommand(ros::NodeHandle& nh, 
                std::string group_name, 
                std::vector<double> &joint_position_, 
                int tab)
{
    // 创建发布者
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/" + group_name + "_command", 20);
    // 设置发布频率
    ros::Rate slow_rate(0.5);  // 0.5 Hz
    ros::Rate fast_rate(50);  // 50 Hz

    // 加载机器人模型（URDF + SRDF）
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    // 创建 PlanningScene 对象
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1:" << group_name <<"Current state is " << (collision_result.collision ? "in" : "not in")
                                                        << " self collision");
    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    // 创建 RobotState 对象
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    robot_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    // 获取初始位置
    std::vector<double> group_position;
    robot_state->copyJointGroupPositions(joint_model_group, group_position);
    // moveit规划接口
    moveit::planning_interface::MoveGroupInterface move_group(group_name);

    std::queue<std::vector<double>> joint_que;
    joint_que.push(group_position);

    bool model_turn = true;

    while (ros::ok()) {
        //更新关节角度
        for(int i=0; i<group_position.size(); i++){
            group_position[i] = joint_position_[i + tab];
        }
        // 设置关节角度
        robot_state->setJointGroupPositions(joint_model_group, group_position);
        // 关节限制
        robot_state->enforceBounds(); 
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        // 检查自碰撞
        current_state.setJointGroupPositions(joint_model_group, joint_values);
        collision_result.clear();
        planning_scene.checkSelfCollision(collision_request, collision_result);

        if (collision_result.collision)
        {
            ROS_WARN("Self-collision detected!");
            // 发生碰撞时，仍然发送队尾中的位置
            // model_turn = true;
            command_model = 0;
            continue;
        }
        else{
            // 模式控制为全局变量
            if(command_model == 0){
                ROS_INFO("moveit control");
                // 用moveit规划接口执行运动
                move_group.setJointValueTarget(joint_values);
                // move_group.asyncMove();
                move_group.move();
                // 更新队列
                joint_que.push(joint_values);
                if(joint_que.size() > 1){
                    joint_que.pop();
                }
                
                // slow_rate.sleep();
                model_turn = true;
            }
            else if(command_model == 1){
                ROS_INFO("command control");
                if(model_turn){
                    // 用moveit规划接口执行运动
                    move_group.setJointValueTarget(joint_values);
                    // move_group.asyncMove();
                    move_group.move();
                    // 更新队列
                    joint_que.push(joint_values);
                    if(joint_que.size() > 1){
                        joint_que.pop();
                    }

                    model_turn = false;
                }
                while(1){
                    if(joint_que.empty()){
                        ROS_ERROR("Joint queue is empty!");
                        break;
                    }
                    std::vector<double> joint_last = joint_que.back();
                    std::vector<double> joint_now(joint_last.size(), 0.0);
                    bool areEqual = true;
                    for(int i=0; i<joint_values.size(); i++){
                        if(std::abs(joint_last[i] - joint_values[i]) > 0.0001){
                            areEqual = false;
                            if(joint_values[i] - joint_last[i]> 0.05){
                                joint_now[i] = joint_last[i] + 0.05;
                            }
                            else if(joint_values[i] - joint_last[i] < -0.05){
                                joint_now[i] = joint_last[i] - 0.05;
                            }
                            else{
                                joint_now[i] = joint_values[i];
                            }
                        }
                        else{
                            joint_now[i] = joint_values[i];
                        }
                    }
                    //关节没有变化，跳出
                    if(areEqual){
                        break;
                    }
                    // 如果关节变化，入队
                    joint_que.push(joint_now);
                }

                //直接发布关节命令
                // 创建JointState消息
                sensor_msgs::JointState joint_state;
                // 设置时间戳
                joint_state.header.stamp = ros::Time::now();
                // 设置关节名称
                joint_state.name = joint_names;
                // 设置关节位置
                std::vector<double> joint_result = joint_que.front();
                joint_state.position = joint_result;
                // 发布关节状态
                joint_state_pub.publish(joint_state);

                // 队列
                if(joint_que.size() > 1){
                    joint_que.pop();
                }

                fast_rate.sleep();
            }
        }

    }

}


int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "moveit_test");
    ros::NodeHandle nh;
    // 订阅模式转换命令
    ros::Subscriber sub = nh.subscribe("mode_topic", 1000, modelCallback);
    
    // 全部关节
    std::vector<double> joint_position_(20, 0.0);
    

    // 新建单独接收数据的线程
    std::thread receiver_thread(receive, std::ref(joint_position_));
    receiver_thread.detach();

    // 新建左臂关节命令发送线程
    std::thread left_arm_thread(sendCommand,std::ref(nh), "left_arm", std::ref(joint_position_), 4);
    left_arm_thread.detach();

    // // 新建右臂关节命令发送线程
    // std::thread right_arm_thread(sendCommand,std::ref(nh), "right_arm", std::ref(joint_position_), 13);
    // right_arm_thread.detach();
    
    ros::spin();
    
    return 0;
}