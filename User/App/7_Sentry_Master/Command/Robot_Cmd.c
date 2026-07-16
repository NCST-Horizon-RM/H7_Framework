//
// Created by CaoKangqi on 2026/6/23.
//
#include "Robot_Cmd.h"
#include "Message_Center.h"
#include "System_State.h"
#include "SBUS.h"
#include "Aim_Vision.h"
#include "All_define.h"
#include "BSP_UART.h"
#include "Horizon_MATH.h"
#include "Comm_DualBoard.h"
#include "Referee.h"
#include "usart.h"
#include "VT13.h"
#include "Robot_Config.h"

#define PITCH_MAX              25.0f
#define PITCH_MIN             -20.0f
#define FRICTION_MAX_RPM       6500.0f
#define FRICTION_RAMP_STEP     1.7f    //摩擦轮缓启动时长

#define RC_ROCKER_XY_COEF      0.004f  // 摇杆控制平移的增益
#define RC_ROCKER_VW_COEF      0.02f   // 摇杆控制自旋的增益
#define RC_PITCH_COEF          0.001f
#define RC_YAW_COEF            0.006f

#define KB_WASD_COEF           1.0f    // 键盘 WASD 速度增益
#define MOUSE_PITCH_COEF       0.06f
#define MOUSE_YAW_COEF         0.04f

#define YAW_ZERO               0.0f

// --- Pub/Sub 句柄 ---
static Subscriber_t *sys_state_sub;
static Subscriber_t *sbus_sub;
static Subscriber_t *vt13_sub;
static Subscriber_t *referee_sub;
static Subscriber_t *gimbal_motors_sub;

static Publisher_t *chassis_cmd_pub;
static Publisher_t *gimbal_cmd_pub;

// --- 本地静态内存缓存 ---
static System_State_t cmd_sys_state;
static SBUS_DATA_typedef sbus_data;
static VT13_Typedef vt13_data;
static Referee_Data_t referee_data;
static Gimbal_Motor_Group_t gimbal_motors_data;

static Chassis_Cmd_t chassis_cmd = {0};
static Gimbal_Cmd_t gimbal_cmd = {0};

// --- 私有函数声明 ---
static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_DualBoard_Sync(void);


void Robot_Cmd_Init(void)
{
    sys_state_sub = SubRegister("system_state", sizeof(System_State_t));
    sbus_sub      = SubRegister("sbus_data", sizeof(SBUS_DATA_typedef));
    vt13_sub     = SubRegister("vt13_data", sizeof(VT13_Typedef));
    referee_sub  = SubRegister("referee_data", sizeof(Referee_Data_t));
    gimbal_motors_sub = SubRegister("gimbal_motors", sizeof(Gimbal_Motor_Group_t));

    chassis_cmd_pub = PubRegister("chassis_cmd", &chassis_cmd, sizeof(Chassis_Cmd_t));
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", &gimbal_cmd, sizeof(Gimbal_Cmd_t));
}

void Robot_Cmd_Update(void)
{
    if (sys_state_sub) SubGetMessage(sys_state_sub, &cmd_sys_state);
    if (sbus_sub)      SubGetMessage(sbus_sub, &sbus_data);
    if (vt13_sub)     SubGetMessage(vt13_sub, &vt13_data);
    if (referee_sub)   SubGetMessage(referee_sub, &referee_data);
    if (gimbal_motors_sub) SubGetMessage(gimbal_motors_sub, &gimbal_motors_data);

    System_State_Report_Remote(vt13_data.offline.is_online || sbus_data.offline.is_online);//向系统状态模块传入遥控器在线状态

    if (cmd_sys_state.global_mode == GLOBAL_SAFE_LOCK ||
        cmd_sys_state.global_mode == GLOBAL_MODULE_ERROR ||
        cmd_sys_state.global_mode == GLOBAL_STANDBY)
    {
        Cmd_Handle_Safe_Mode();
    }

    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd);

    // 双板通信
    Cmd_DualBoard_Sync();
}

/**
 * @brief 安全模式清除物理输出
 */
static void Cmd_Handle_Safe_Mode(void)
{
    chassis_cmd.mode = CHASSIS_CMD_SAFE;
    gimbal_cmd.mode  = GIMBAL_CMD_SAFE;

    chassis_cmd.target_vx = 0.0f;
    chassis_cmd.target_vy = 0.0f;
    chassis_cmd.target_vw = 0.0f;

}

/**
 * @brief 遥控器模式
 */
static void Cmd_Update_Remote_Ctrl(void)
{
    int16_t relative_angle1 = YAW_ZERO - (float)gimbal_motors_data.DM4310_Yaw.Angle_now;
    if (relative_angle1 > 4096) {relative_angle1 -= 8192;}
    else if (relative_angle1 < -4096) {relative_angle1 += 8192;}
    chassis_cmd.offset_angle_chassis = (float)relative_angle1 * ENCODER_TO_RAD;

    int16_t relative_angle2 = YAW_ZERO - (float)gimbal_motors_data.DM4310_Yaw.Angle_now;
    if (relative_angle2 > 4096) {relative_angle2 -= 8192;}
    else if (relative_angle2 < -4096) {relative_angle2 += 8192;}
    gimbal_cmd.offset_angle_YAW = (float)relative_angle2 * ENCODER_TO_RAD;

    chassis_cmd.target_vx = (float)SBUS_GetChannelValue(&sbus_data, SBUS_Channel_1) * RC_ROCKER_XY_COEF;
    chassis_cmd.target_vy = (float)SBUS_GetChannelValue(&sbus_data, SBUS_Channel_2) * RC_ROCKER_XY_COEF;
    //chassis_cmd.target_vw = (float)SBUS_GetChannelValue(&sbus_data, SBUS_Channel_3) * RC_ROCKER_VW_COEF + (float)vt13_data.Remote.Channel[3] * RC_ROCKER_VW_COEF;
    gimbal_cmd.target_yaw   -= (float)SBUS_GetChannelValue(&sbus_data, SBUS_Channel_4) * RC_YAW_COEF*DEG2RAD;
    gimbal_cmd.target_pitch   = (float)SBUS_GetChannelValue(&sbus_data, SBUS_Channel_3) * RC_PITCH_COEF*DEG2RAD;

    normalize_to_pi(gimbal_cmd.target_yaw);
    normalize_to_pi(gimbal_cmd.target_pitch);

    static bool Key = 0;
    if (SBUS_GetChannelState(&sbus_data, SBUS_Channel_7) == SBUS_SW_UP&&(Key==1||referee_data.game_status.game_progress==4)) {

        Key=0;
    }
    else if ((referee_data.game_status.game_progress==4&&referee_data.game_robot_HP.ally_7_robot_HP==0)||(referee_data.game_status.game_progress!=4&&SBUS_GetChannelState(&sbus_data, SBUS_Channel_7) == SBUS_SW_Cen&&Key==0) ){//

        Key=1;
        //Gimbal_Master.yaw_target = Gyro.yaw;
    }
    else if ((referee_data.game_status.game_progress==4&&referee_data.game_robot_HP.ally_7_robot_HP!=0)||(referee_data.game_status.game_progress!=4&&SBUS_GetChannelState(&sbus_data, SBUS_Channel_7) == SBUS_SW_Down&&Key==1)) {//

        Key=0;
    }
}
/**
 * @brief 双板数据同步逻辑
 */
static void Cmd_DualBoard_Sync(void)
{

    // DualBoard_Send(LINK_CAN, &Tx_Data, sizeof(B2B_Tx_t));
}