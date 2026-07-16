//
// Created by CaoKangqi on 2026/6/23.
// 模块功能：总控中心逻辑（纯净版）- 指令集与接口
//
#ifndef H7_FRAMEWORK_ROBOT_CMD_H
#define H7_FRAMEWORK_ROBOT_CMD_H

#include <stdint.h>
#include <stdbool.h>

// 底盘控制指令
typedef enum {
    CHASSIS_CMD_SAFE = 0,    // 安全锁死，无输出
    CHASSIS_CMD_FOLLOW,      // 底盘跟随云台
    CHASSIS_CMD_DIRECT,        // 底盘跟随速度方向
    CHASSIS_CMD_SPIN         // 小陀螺模式
} Chassis_Mode_e;

typedef struct {
    Chassis_Mode_e mode;
    float target_vx;         // 目标 X 轴平移速度 (m/s)
    float target_vy;         // 目标 Y 轴平移速度 (m/s)
    float target_vw;         // 目标自旋角速度 (rad/s)
    float offset_angle_chassis;      // 云台与底盘的相对夹角
} Chassis_Cmd_t;

// 台控制指令
typedef enum {
    GIMBAL_CMD_SAFE = 0,     // 安全锁死
    GIMBAL_CMD_MANUAL,       // 键鼠/遥控器控制
    GIMBAL_CMD_AUTO_AIM,      // 视觉自瞄控制
    GIMBAL_CMD_SEARCH         // 巡逻
} Gimbal_Mode_e;

typedef struct {
    Gimbal_Mode_e mode;
    float target_pitch;      // 目标 Pitch 角度
    float target_yaw;
    float target_Yaw;        // 目标 Yaw 角度
    float offset_angle_YAW;
} Gimbal_Cmd_t;

void Robot_Cmd_Init(void);
void Robot_Cmd_Update(void);

#endif //H7_FRAMEWORK_ROBOT_CMD_H