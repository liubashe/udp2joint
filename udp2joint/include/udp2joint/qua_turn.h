#ifndef QUA_TURN
#define QUA_TURN
#include <stdio.h>
#include <iostream>
#include <cmath>

using namespace std;

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
    double ang_6 = atan2(sqrt(p_q.y*p_q.y+p_q.z*p_q.z), abs(p_q.x));


    return {ang_1, ang_2, spin_angle, ang_4, ang_5, ang_6, spin_angle};  // 返回旋转轴 (x, y, z) 和旋转角度
}

#endif