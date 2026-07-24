#ifndef __UI_H
#define __UI_H

#include "stm32h7xx_hal.h"
#include "rp_math.h"
#include "ui_protocol.h"
#include "ui_priority.h"
#include "Balance.h"
#include "Chassis.h"
#include "car_info.h"
#include "gimbal.h"
#include "communicate.h"
#include "cap.h"

typedef enum {
    CHAS_HEAD_LINE,
    CHAS_SIDE_LINE,
    CAP_LINE,
    VISION_AIM,
    AUTO_CATCH_FRAME,
    LENGTH_FRAME,
    D_CAR_MODE,
    L_LEG_BODY_LINE,
    L_LEG_A_TO_D,
    L_LEG_D_TO_C,
    R_LEG_BODY_LINE,
    R_LEG_A_TO_D,
    R_LEG_D_TO_C,
    L_LEG_BODY_BACK_CIRCLE,
    L_LEG_BODY_FRONT_CIRCLE,
    L_LEG_C_CIRCLE,
    R_LEG_BODY_BACK_CIRCLE,
    R_LEG_BODY_FRONT_CIRCLE,
    R_LEG_C_CIRCLE,
    D_RED_1_HEALTH_CHAR,
    D_RED_2_HEALTH_CHAR,
    D_RED_3_HEALTH_CHAR,
    D_RED_4_HEALTH_CHAR,
    D_RED_5_HEALTH_CHAR,
    D_BLUE_1_HEALTH_CHAR,
    D_BLUE_2_HEALTH_CHAR,
    D_BLUE_3_HEALTH_CHAR,
    D_BLUE_4_HEALTH_CHAR,
    D_BLUE_5_HEALTH_CHAR,
    DART_WARNING_CHAR,
    D_L_FRIC_STATE_CYCLE,
    D_R_FRIC_STATE_CYCLE,
    D_VISION_DETECT_ROBOT_HEALTH_INT,
    D_ENERMY_MONEY_INT,
    DYNAMIC_NUM,
} dynamic_ui_e;

typedef enum {
    CHAS_CIRCLE,
    CAP_FRAME,
    LOW_CHAR,
    MID_CHAR,
    HIGH_CHAR,
    C_CAR_MODE_CHAR,
    C_RED_1_CHAR,
    C_RED_2_CHAR,
    C_RED_3_CHAR,
    C_RED_4_CHAR,
    C_RED_5_CHAR,
    C_BLUE_1_CHAR,
    C_BLUE_2_CHAR,
    C_BLUE_3_CHAR,
    C_BLUE_4_CHAR,
    C_BLUE_5_CHAR,
    C_Fric_CHAR,
    CONST_NUM,
} const_ui_e;

void My_Ui_Init(void);
void Ui_Info_Update(void);
void update_robot_health(void);
void update_dart_warning(void);
void update_fric_state_cycles(void);
void update_vision_detect_robot_health(void);
void update_enermy_money(void);

/*==================== Leg UI Begin ====================*/
// 腿部UI配置参数结构体
typedef struct {
    float scale;            // 放大比例
    int16_t leg_offset_x;   // 左腿整体偏置X
    int16_t leg_offset_y;   // 左腿整体偏置Y
    int16_t right_offset_x; // 右腿相对左腿偏置X
    int16_t right_offset_y; // 右腿相对左腿偏置Y
    int16_t body_length;    // 机体杆半长度

} Leg_UI_Config_t;

// 腿部UI中间变量结构体
typedef struct {
    float pitch; // 机体pitch角度
    // 左腿原始坐标（取负后）
    float raw_A_l_x, raw_A_l_y;
    float raw_D_l_x, raw_D_l_y;
    float raw_C_l_x, raw_C_l_y;
    // 右腿原始坐标（取负后）
    float raw_A_r_x, raw_A_r_y;
    float raw_D_r_x, raw_D_r_y;
    float raw_C_r_x, raw_C_r_y;
    // 左腿世界坐标（旋转后）
    float world_A_l_x, world_A_l_y;
    float world_D_l_x, world_D_l_y;
    float world_C_l_x, world_C_l_y;
    // 右腿世界坐标（旋转后）
    float world_A_r_x, world_A_r_y;
    float world_D_r_x, world_D_r_y;
    float world_C_r_x, world_C_r_y;
    // 机体杆端点（左腿）
    float body_back_l_x, body_back_l_y;
    float body_front_l_x, body_front_l_y;
    // 机体杆端点（右腿）
    float body_back_r_x, body_back_r_y;
    float body_front_r_x, body_front_r_y;
} Leg_UI_Var_t;

/*==================== Leg UI End ====================*/

#endif
