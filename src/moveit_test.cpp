#include <stdio.h>
#include <iostream>
#include <thread>
#include <vector>
#include <queue>

#include "VDMocapSDK_DataRead_DataType.h"  //数据类型头文件
#include "time.h"
#include "MySocketUdp.h"
#include "PositionB_FK.h"
#include "PositionH_FK.h"
#include "Dataanalysis.h"
#include "qua_turn.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// 添加controller_manager_msgs头文件
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std;

class MoveitTest : public rclcpp::Node
{
public:
    MoveitTest() 
        : Node("moveit_test"),
          robot_model_loader_(
              std::shared_ptr<rclcpp::Node>(this),  // 将this转换为shared_ptr
              "robot_description"
          )
    {
        // 将全局变量和回调函数移到类内部
        command_model_ = 0;
        joint_current_ = std::vector<double>(20, 0.0);
        joint_position_ = std::vector<double>(20, 0.0);
        
        // 修改订阅者创建方式
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "mode_topic", 10,
            std::bind(&MoveitTest::modelCallback, this, std::placeholders::_1)
        );
         // 创建订阅者，订阅joint_states话题
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",  // 话题名称
            10,              // 队列大小
            std::bind(&MoveitTest::joint_state_callback, this, std::placeholders::_1));
        // 创建切换控制器的客户端
        switch_controller_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
        while (!switch_controller_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for switch_controller service...");
        }
        // //切换为轨迹控制器
        // switchToTrajectoryController();
        // 切换为位置控制器
        switchToPositionController();
        // 获取机器人模型
        kinematic_model_ = robot_model_loader_.getModel();
        
        // 修改线程创建方式
        receiver_thread_ = std::thread(&MoveitTest::receive, this);
        // left_arm_thread_ = std::thread(&MoveitTest::sendJointState, this);
        left_arm_thread_ = std::thread(&MoveitTest::loopControl, this, "left_arm", 4);
        // right_arm_thread_ = std::thread(&MoveitTest::sendCommand, this, "right_arm", 13);
        receiver_thread_.detach();
        left_arm_thread_.detach();
        // right_arm_thread_.detach();
    }

private:
    void modelCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        command_model_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "model: %d", command_model_);
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for(int i=0; i<msg->name.size(); i++){
            //对应到joint_names_中的索引
            for(int j=0; j<joint_names_.size(); j++){
                if(msg->name[i] == joint_names_[j]){
                    joint_current_[j] = msg->position[i];
                    break;
                }
            }
        }
    }
    
    // 关节限制
    double joint_limit(double delta){
        if(delta < 0.06){
            return std::abs(delta + 0.01);
        }
        else{
            return 0.06;
        }
    }

    void loopControl(std::string group_name, int tab)
    {
        // 修改发布者创建方式
        auto command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/" + group_name + "_position_controller/commands", 10);        
        
        // 设置发布频率
        rclcpp::Rate loop_rate(100);  // 10 Hz

        // 创建 PlanningScene 对象
        planning_scene::PlanningScene planning_scene(kinematic_model_);
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        planning_scene.checkSelfCollision(collision_request, collision_result);
        RCLCPP_INFO(this->get_logger(), "Test 1: %s Current state is %s self collision",
            group_name.c_str(), (collision_result.collision ? "in" : "not in"));
        moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
        
        // 创建 RobotState 对象
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model_));
        robot_state->setToDefaultValues();
        const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        
        // 获取初始位置
        std::vector<double> group_position;
        robot_state->copyJointGroupPositions(joint_model_group, group_position);
        std::vector<double> group_current;
        robot_state->copyJointGroupPositions(joint_model_group, group_current);

        // 关节限制
        std::vector<double> joint_delta;
        joint_delta.resize(group_position.size(), 0.0);
        // double joint_limit = 0.05;
        
        while(rclcpp::ok()){
            // 更新关节目标角度
            for(int i=0; i<group_position.size(); i++){
                group_position[i] = joint_position_[i + tab];
            }
            // 更新关节当前角度
            for(int i=0; i<group_current.size(); i++){
                group_current[i] = joint_current_[i + tab];
            }
            // 关节变化量限制
            for(int i=0; i<group_position.size(); i++){
                if((group_position[i] - group_current[i]) > joint_limit(joint_delta[i])){
                    group_position[i] = group_current[i] + joint_limit(joint_delta[i]);
                }
                else if((group_position[i] - group_current[i]) < -joint_limit(joint_delta[i])){
                    group_position[i] = group_current[i] - joint_limit(joint_delta[i]);
                }
                joint_delta[i] = std::abs(group_position[i] - group_current[i]);
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
            if(collision_result.collision){
                RCLCPP_WARN(this->get_logger(), "Self-collision detected!");
                continue;
            }   
            else{
                // RCLCPP_INFO(this->get_logger(), "No self-collision detected!");
                // 发布关节命令
                auto joint_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
                joint_msg->data = joint_values;
                command_pub_->publish(std::move(joint_msg));

            }
            loop_rate.sleep();
        }
    }
    void sendJointState()
    {
        // 修改发布者创建方式
        auto joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState> ("joint_states", 20);

        // 设置发布频率
        rclcpp::Rate loop_rate(60);  // 10 Hz
        // 添加关节名称
        std::vector<string> joint_names;
        joint_names = {
            "Body_Joint1","Body_Joint2","Body_Joint3","Body_Joint4",
            "Left_Arm_Joint1","Left_Arm_Joint2","Left_Arm_Joint3","Left_Arm_Joint4","Left_Arm_Joint5","Left_Arm_Joint6","Left_Arm_Joint7",
            "Neck_Joint1","Neck_Joint2",
            "Right_Arm_Joint1","Right_Arm_Joint2","Right_Arm_Joint3","Right_Arm_Joint4","Right_Arm_Joint5","Right_Arm_Joint6","Right_Arm_Joint7"
        };

        while(rclcpp::ok()){


            // 创建JointState消息
            sensor_msgs::msg::JointState joint_state;
            // 设置时间戳
            joint_state.header.stamp = this->now();
            // 设置关节名称
            joint_state.name = joint_names;
            // 设置关节位置
            joint_state.position = joint_position_;
            // 发布关节状态
            joint_state_pub->publish(joint_state);
            
            loop_rate.sleep();
        }            
    }
        // 切换到位置控制器
    bool switchToPositionController()
    {
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        
        // 使用新的字段名称
        request->activate_controllers = {"left_arm_position_controller", "right_arm_position_controller"};
        request->deactivate_controllers = {"left_arm_controller", "right_arm_controller"};
        request->activate_asap = true;
        request->strictness = request->BEST_EFFORT;
        request->timeout.sec = 2;
        
        try {
            auto result_future = switch_controller_client_->async_send_request(request);
            
            if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = result_future.get();
                if (result->ok) {
                    RCLCPP_INFO(this->get_logger(), "Switch to position controller success");
                    return true;
                }
            }
            RCLCPP_ERROR(this->get_logger(), "Switch to position controller failed");
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during controller switch: %s", e.what());
            return false;
        }
    }

    // 切换到轨迹控制器
    bool switchToTrajectoryController()
    {
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        
        // 使用新的字段名称
        request->activate_controllers = {"left_arm_controller", "right_arm_controller"};
        request->deactivate_controllers = {"left_arm_position_controller", "right_arm_position_controller"};
        request->activate_asap = true;
        request->strictness = request->BEST_EFFORT;
        request->timeout.sec = 2;
        
        try {
            auto result_future = switch_controller_client_->async_send_request(request);
            
            if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = result_future.get();
                if (result->ok) {
                    RCLCPP_INFO(this->get_logger(), "Switch to trajectory controller success");
                    return true;
                }
            }
            RCLCPP_ERROR(this->get_logger(), "Switch to trajectory controller failed");
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during controller switch: %s", e.what());
            return false;
        }
    }

    void sendCommand(std::string group_name, int tab)
    {
        // 修改发布者创建方式
        // auto joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        //     "/" + group_name + "_command", 20);
        auto command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/" + group_name + "_position_controller/commands", 10);
            
        // 修改rate创建方式
        rclcpp::Rate slow_rate(0.5);
        rclcpp::Rate fast_rate(50);

        // 创建 PlanningScene 对象
        planning_scene::PlanningScene planning_scene(kinematic_model_);
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        planning_scene.checkSelfCollision(collision_request, collision_result);
        RCLCPP_INFO(this->get_logger(), "Test 1: %s Current state is %s self collision",
            group_name.c_str(), (collision_result.collision ? "in" : "not in"));
        moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
        
        // 创建 RobotState 对象
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model_));
        robot_state->setToDefaultValues();
        const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        
        // 获取初始位置
        std::vector<double> group_position;
        robot_state->copyJointGroupPositions(joint_model_group, group_position);
        
        // moveit规划接口 - 修改构造函数调用
        moveit::planning_interface::MoveGroupInterface move_group(
            std::shared_ptr<rclcpp::Node>(this),  // 添加节点指针
            group_name
        );

        std::queue<std::vector<double>> joint_que;
        joint_que.push(group_position);

        bool model_turn = true;//运动切换标志

        while (rclcpp::ok()) {
            //更新关节目标角度
            for(int i=0; i<group_position.size(); i++){
                group_position[i] = joint_position_[i + tab];
            }
            //更新关节当前角度

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
                RCLCPP_WARN(this->get_logger(), "Self-collision detected!");
                // 发生碰撞时，仍然发送队尾中的位置
                if(command_model_ == 1){
                    // model_turn = true;
                    command_model_ = 0;
                    //切换为轨迹控制器
                    switchToTrajectoryController();
                }
                continue;
            }
            else{
                // 模式控制为全局变量
                if(command_model_ == 0){
                    RCLCPP_INFO(this->get_logger(), "moveit control");
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
                else if(command_model_ == 1){
                    RCLCPP_INFO(this->get_logger(), "command control");
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
                        //切换为位置控制器
                        switchToPositionController();
                        model_turn = false;
                    }
                    while(1){
                        if(joint_que.empty()){
                            RCLCPP_ERROR(this->get_logger(), "Joint queue is empty!");
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

                    // //直接发布关节命令
                    // // 创建JointState消息
                    // sensor_msgs::msg::JointState joint_state;
                    // // 设置时间戳
                    // joint_state.header.stamp = this->now();
                    // // 设置关节名称
                    // joint_state.name = joint_names;
                    // // 设置关节位置
                    // std::vector<double> joint_result = joint_que.front();
                    // joint_state.position = joint_result;
                    // // 发布关节状态
                    // joint_state_pub->publish(joint_state);

                    auto joint_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
                    std::vector<double> joint_result = joint_que.front();
                    joint_msg->data = joint_result;
                    command_pub_->publish(std::move(joint_msg));

                    // 队列
                    if(joint_que.size() > 1){
                        joint_que.pop();
                    }

                    fast_rate.sleep();
                }
            }

        }
    }

    void Copy(unsigned char *out, char *in, unsigned length){
        for (int i = 0; i < length; i++){
            out[i] = (unsigned char)(in[i]);
        }
    }

    void receive()
    {
        //远端ip（VDMocapStudio软件所在电脑的本地ip，在VDMocapStudio软件中有显示）
        const char* dst_ip = "192.168.26.95"; //IP地址自行修改
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

        while(rclcpp::ok())
        {
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
            joint_position_[7]= (-2.4+lud_angles[3]);
            if(joint_position_[7] < -3) joint_position_[7] = 0.6;
            //左手
            Quaternion q_ldh = quaternion_relative(qld, qlh);
            std::vector<double> ldh_angles = quaternion_to_axis_angle(q_ldh, ql);
            joint_position_[8]= (0.5-ldh_angles[4]);
            joint_position_[9]= (-ldh_angles[5]);
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
        delete myUdp;  // 清理资源
    }
    // 类成员变量
    int command_model_;
    std::vector<double> joint_position_;
    std::vector<double> joint_current_;
    std::vector<string> joint_names_ = {
        "Body_Joint1","Body_Joint2","Body_Joint3","Body_Joint4",
        "Left_Arm_Joint1","Left_Arm_Joint2","Left_Arm_Joint3","Left_Arm_Joint4","Left_Arm_Joint5","Left_Arm_Joint6","Left_Arm_Joint7",
        "Neck_Joint1","Neck_Joint2",
        "Right_Arm_Joint1","Right_Arm_Joint2","Right_Arm_Joint3","Right_Arm_Joint4","Right_Arm_Joint5","Right_Arm_Joint6","Right_Arm_Joint7"
    };
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::thread receiver_thread_;
    std::thread left_arm_thread_;
    std::thread right_arm_thread_;
    robot_model_loader::RobotModelLoader robot_model_loader_;  // RobotModelLoader
    moveit::core::RobotModelPtr kinematic_model_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
};

int main(int argc, char **argv)
{
    std::this_thread::sleep_for(std::chrono::seconds(10));  // 延时10秒

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}