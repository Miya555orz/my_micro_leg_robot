#include "ui.h"
#include "Balance.h"
#include "Chassis.h"
#include "arm_math.h"
#include "cap.h"
#include "car_info.h"
#include "chassis_motor.h"
#include "communicate.h"
#include "gimbal.h"
#include "ui_priority.h"
#include "ui_protocol.h"
#include <string.h>

/*==================== Leg UI Variables Begin ====================*/
Leg_UI_Config_t Leg_UI_Config = {
    .scale = 430.0f,
    .leg_offset_x = 1550,
    .leg_offset_y = 485,
    .right_offset_x = 200,
    .right_offset_y = 0,
    .body_length = 50, // 半长度
};

Leg_UI_Var_t Leg_UI_Var;
/*==================== Leg UI Variables End ====================*/

#define CHAS_CIRCLE_X (Client_mid_position_x)
#define CHAS_CIRCLE_Y (Client_mid_position_y - 350)
#define CHAS_CIRCLE_R 70

// 腿部UI
#define Sd_Circle_Radius 17
#define Wheel_Circle_Radius 19

// 血量UI
#define ROBOT_NUM_UI_Y 880
#define ROBOT_HEALTH_UI_Y 850
#define RED_ROBOT_1_X 680
#define RED_ROBOT_2_X 565
#define RED_ROBOT_3_X 450
#define RED_ROBOT_4_X 335
#define RED_ROBOT_5_X 220
#define BLUE_ROBOT_1_X 1220
#define BLUE_ROBOT_2_X 1335
#define BLUE_ROBOT_3_X 1450
#define BLUE_ROBOT_4_X 1565
#define BLUE_ROBOT_5_X 1680

// 飞镖预警
#define DART_WARNING_X 850
#define DART_WARNING_Y 740

// 左侧状态栏
#define LEFT_STATUS_CHAR_SIZE 20
#define LEFT_STATUS_MODE_X Client_mid_position_x - 800       // 固定字符x坐标
#define LEFT_STATUS_MODE_Y Client_mid_position_y + 17        // 固定字符y坐标
#define LEFT_STATUS_MODE_VALUE_X Client_mid_position_x - 630 // 动态字符x坐标
#define LEFT_STATUS_Y_STEP 40
#define LEFT_STATUS_Fric_Y LEFT_STATUS_MODE_Y + LEFT_STATUS_Y_STEP
#define LEFT_STATUS_L_Fric_X LEFT_STATUS_MODE_VALUE_X
#define LEFT_STATUS_R_Fric_X LEFT_STATUS_MODE_VALUE_X + 40
ui_info_t dynamic_ui_info[DYNAMIC_NUM] = {
    [CHAS_HEAD_LINE] = {
        .ui_config.priority = HIGH_PRIORITY,
        .ui_config.ui_type = LINE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = CYAN_BLUE,
        .ui_config.width = 4,
        .ui_config.start_x = CHAS_CIRCLE_X,
        .ui_config.start_y = CHAS_CIRCLE_Y,
        .ui_config.end_x = CHAS_CIRCLE_X,
        .ui_config.end_y = CHAS_CIRCLE_Y + CHAS_CIRCLE_R,
    },
    [CHAS_SIDE_LINE] = {
        .ui_config.priority = HIGH_PRIORITY,
        .ui_config.ui_type = LINE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = WHITE,
        .ui_config.width = 2,
        .ui_config.start_x = CHAS_CIRCLE_X - CHAS_CIRCLE_R,
        .ui_config.start_y = CHAS_CIRCLE_Y,
        .ui_config.end_x = CHAS_CIRCLE_X + CHAS_CIRCLE_R,
        .ui_config.end_y = CHAS_CIRCLE_Y,
    },
    [CAP_LINE] = {
        .ui_config.priority = HIGH_PRIORITY,
        .ui_config.ui_type = LINE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 1,
        .ui_config.color = GREEN,
        .ui_config.width = 25,
        .ui_config.start_x = Client_mid_position_x - 250,
        .ui_config.start_y = Client_mid_position_y + 332,
        .ui_config.end_x = Client_mid_position_x + 250,
        .ui_config.end_y = Client_mid_position_y + 332,
    },
    [VISION_AIM] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CIRCLE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 1,
        .ui_config.color = WHITE,
        .ui_config.width = 1,
        .ui_config.start_x = Client_mid_position_x,
        .ui_config.start_y = Client_mid_position_y,
        .ui_config.radius = 3,
    },
    [AUTO_CATCH_FRAME] = {
        .ui_config.priority = HIGH_PRIORITY,
        .ui_config.ui_type = RECTANGEL,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 1,
        .ui_config.color = WHITE,
        .ui_config.width = 3,
        .ui_config.start_x = Client_mid_position_x - 280,
        .ui_config.start_y = Client_mid_position_y + 170,
        .ui_config.end_x = Client_mid_position_x + 280,
        .ui_config.end_y = Client_mid_position_y - 180,
    },
    [LENGTH_FRAME] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = RECTANGEL,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 1,
        .ui_config.color = YELLOW,
        .ui_config.width = 3,
        .ui_config.start_x = Client_mid_position_x - 350,
        .ui_config.start_y = Client_mid_position_y - 125,
        .ui_config.end_x = Client_mid_position_x - 310,
        .ui_config.end_y = Client_mid_position_y - 165,
    },
    [D_CAR_MODE] = {
        /*不变配置*/
        .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
        .ui_config.ui_type = CHAR,          // UI内容类型
        /*可变配置*/
        .ui_config.operate_type = MODIFY,                 // 操作类型
        .ui_config.layer = 1,                             // 图层数，0~9
        .ui_config.color = GREEN,                         // 颜色
        .ui_config.size = LEFT_STATUS_CHAR_SIZE,          // 字体大小
        .ui_config.width = 2,                             // 线条宽度
        .ui_config.start_x = Client_mid_position_x - 630, // 起点 x 坐标
        .ui_config.start_y = Client_mid_position_y + 17,  // 起点 y 坐标
        .ui_config.text = "Hank Liang",                   // 显示的文字
    },
    /* 左腿线段 begin */
    [L_LEG_BODY_LINE] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = LINE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.width = 3,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.end_x = 0,
        .ui_config.end_y = 0,
    },
    [L_LEG_A_TO_D] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = LINE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = CYAN_BLUE,
        .ui_config.width = 3,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.end_x = 0,
        .ui_config.end_y = 0,
    },
    [L_LEG_D_TO_C] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = LINE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = CYAN_BLUE,
        .ui_config.width = 3,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.end_x = 0,
        .ui_config.end_y = 0,
    },
    /* 左腿线段 end */
    /* 右腿线段 begin */
    [R_LEG_BODY_LINE] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = LINE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = WHITE,
        .ui_config.width = 3,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.end_x = 0,
        .ui_config.end_y = 0,
    },
    [R_LEG_A_TO_D] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = LINE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = CYAN_BLUE,
        .ui_config.width = 3,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.end_x = 0,
        .ui_config.end_y = 0,
    },
    [R_LEG_D_TO_C] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = LINE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = CYAN_BLUE,
        .ui_config.width = 3,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.end_x = 0,
        .ui_config.end_y = 0,
    },
    /* 右腿线段 end */
    /* 左腿圆点 begin */
    [L_LEG_BODY_BACK_CIRCLE] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CIRCLE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = GREEN,
        .ui_config.width = 2,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.radius = Sd_Circle_Radius,
    },
    [L_LEG_BODY_FRONT_CIRCLE] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CIRCLE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = GREEN,
        .ui_config.width = 2,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.radius = Sd_Circle_Radius,
    },
    [L_LEG_C_CIRCLE] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CIRCLE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = GREEN,
        .ui_config.width = 2,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.radius = Wheel_Circle_Radius,
    },
    /* 左腿圆点 end */
    /* 右腿圆点 begin */
    [R_LEG_BODY_BACK_CIRCLE] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CIRCLE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = GREEN,
        .ui_config.width = 2,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.radius = Sd_Circle_Radius,
    },
    [R_LEG_BODY_FRONT_CIRCLE] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CIRCLE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = GREEN,
        .ui_config.width = 2,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.radius = Sd_Circle_Radius,
    },
    [R_LEG_C_CIRCLE] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CIRCLE,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 2,
        .ui_config.color = GREEN,
        .ui_config.width = 2,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.radius = Wheel_Circle_Radius,
    },
    /* 右腿圆点 end */
    /* 红方血量 UI begin */
    [D_RED_1_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_1_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    [D_RED_2_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_2_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    [D_RED_3_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_3_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    [D_RED_4_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_4_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    [D_RED_5_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_5_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    /* 红方血量 UI end */
    /* 蓝方血量 UI begin */
    [D_BLUE_1_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_1_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    [D_BLUE_2_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_2_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    [D_BLUE_3_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_3_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    [D_BLUE_4_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_4_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    [D_BLUE_5_HEALTH_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_5_X,
        .ui_config.start_y = ROBOT_HEALTH_UI_Y,
        .ui_config.text = "0",
    },
    /* 蓝方血量 UI end */
    /* 飞镖预警 UI begin */
    [DART_WARNING_CHAR] = {
        .ui_config.priority = MID_PRIORITY,
        .ui_config.ui_type = CHAR,
        .ui_config.operate_type = MODIFY,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 25,
        .ui_config.width = 3,
        .ui_config.start_x = 0,
        .ui_config.start_y = 0,
        .ui_config.text = "!!!DART!!!",
    },
    /* 飞镖预警 UI end */
    [D_L_FRIC_STATE_CYCLE] = {
        /*不变配置*/
        .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
        .ui_config.ui_type = CIRCLE,        // UI内容类型
        /*可变配置*/
        .ui_config.operate_type = MODIFY,            // 操作类型
        .ui_config.layer = 1,                        // 图层数，0~9
        .ui_config.color = WHITE,                    // 颜色
        .ui_config.width = 20,                       // 线条宽度
        .ui_config.start_x = LEFT_STATUS_L_Fric_X,   // 圆心 x 坐标
        .ui_config.start_y = LEFT_STATUS_Fric_Y - 7, // 圆心 y 坐标
        .ui_config.radius = 7,                       // 半径
    },
    [D_R_FRIC_STATE_CYCLE] = {
        /*不变配置*/
        .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
        .ui_config.ui_type = CIRCLE,        // UI内容类型
        /*可变配置*/
        .ui_config.operate_type = MODIFY,            // 操作类型
        .ui_config.layer = 1,                        // 图层数，0~9
        .ui_config.color = WHITE,                    // 颜色
        .ui_config.width = 20,                       // 线条宽度
        .ui_config.start_x = LEFT_STATUS_R_Fric_X,   // 圆心 x 坐标
        .ui_config.start_y = LEFT_STATUS_Fric_Y - 7, // 圆心 y 坐标
        .ui_config.radius = 7,                       // 半径
    },
    [D_VISION_DETECT_ROBOT_HEALTH_INT] = {
        /*不变配置*/
        .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
        .ui_config.ui_type = INT,            // UI内容类型
        /*可变配置*/
        .ui_config.operate_type = MODIFY, // 操作类型
        .ui_config.layer = 0,             // 图层数，0~9
        .ui_config.color = PINK,          // 颜色
        .ui_config.size = 20,             // 字体大小
        .ui_config.width = 4,             // 线条宽度
        .ui_config.start_x = 935,         // 起点 x 坐标
        .ui_config.start_y = 650,         // 起点 y 坐标
        .ui_config.int_num = 0,           // 显示的数字
    },
    [D_ENERMY_MONEY_INT] = {
        /*不变配置*/
        .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
        .ui_config.ui_type = INT,            // UI内容类型
        /*可变配置*/
        .ui_config.operate_type = MODIFY, // 操作类型
        .ui_config.layer = 0,             // 图层数，0~9
        .ui_config.color = WHITE,         // 颜色
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = 930, // 起点 x 坐标
        .ui_config.start_y = 910, // 起点 y 坐标
        .ui_config.int_num = 0,   // 显示的数字
    },
};

ui_info_t const_ui_info[CONST_NUM] = {
    [CHAS_CIRCLE] = {
        .ui_config.ui_type = CIRCLE,
        .ui_config.layer = 0,
        .ui_config.color = GREEN,
        .ui_config.width = 2,
        .ui_config.start_x = CHAS_CIRCLE_X,
        .ui_config.start_y = CHAS_CIRCLE_Y,
        .ui_config.radius = CHAS_CIRCLE_R,
    },
    [CAP_FRAME] = {
        .ui_config.ui_type = RECTANGEL,
        .ui_config.layer = 1,
        .ui_config.color = WHITE,
        .ui_config.width = 3,
        .ui_config.start_x = Client_mid_position_x - 253,
        .ui_config.start_y = Client_mid_position_y + 345,
        .ui_config.end_x = Client_mid_position_x + 253,
        .ui_config.end_y = Client_mid_position_y + 317,
    },
    [LOW_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 1,
        .ui_config.color = WHITE,
        .ui_config.size = 30,
        .ui_config.width = 2,
        .ui_config.start_x = Client_mid_position_x - 340,
        .ui_config.start_y = Client_mid_position_y - 130,
        .ui_config.text = "L",
    },
    [MID_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 1,
        .ui_config.color = WHITE,
        .ui_config.size = 30,
        .ui_config.width = 2,
        .ui_config.start_x = Client_mid_position_x - 340,
        .ui_config.start_y = Client_mid_position_y - 70,
        .ui_config.text = "M",
    },
    [HIGH_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 1,
        .ui_config.color = WHITE,
        .ui_config.size = 30,
        .ui_config.width = 2,
        .ui_config.start_x = Client_mid_position_x - 340,
        .ui_config.start_y = Client_mid_position_y - 10,
        .ui_config.text = "H",
    },
    [C_CAR_MODE_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = LEFT_STATUS_CHAR_SIZE,
        .ui_config.width = 2,
        .ui_config.start_x = Client_mid_position_x - 800,
        .ui_config.start_y = Client_mid_position_y + 17,
        .ui_config.text = "MODE:",
    },
    [C_RED_1_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_1_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "1",
    },
    [C_RED_2_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_2_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "2",
    },
    [C_RED_3_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_3_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "3",
    },
    [C_RED_4_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_4_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "4",
    },
    [C_RED_5_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = RED_ROBOT_5_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "5",
    },
    [C_BLUE_1_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_1_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "1",
    },
    [C_BLUE_2_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_2_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "2",
    },
    [C_BLUE_3_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_3_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "3",
    },
    [C_BLUE_4_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 1,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_4_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "4",
    },
    [C_BLUE_5_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 3,
        .ui_config.start_x = BLUE_ROBOT_5_X,
        .ui_config.start_y = ROBOT_NUM_UI_Y,
        .ui_config.text = "5",
    },
    [C_Fric_CHAR] = {
        .ui_config.ui_type = CHAR,
        .ui_config.layer = 0,
        .ui_config.color = WHITE,
        .ui_config.size = 20,
        .ui_config.width = 2,
        .ui_config.start_x = LEFT_STATUS_MODE_X,
        .ui_config.start_y = LEFT_STATUS_Fric_Y,
        .ui_config.text = "Fric:",
    },
};

/* 旋转坐标点 */
static void rotate_point_f(float *x, float *y,
                           float raw_x, float raw_y,
                           float mid_x, float mid_y, float angle)
{
    float s = sin(angle);
    float c = cos(angle);
    float origin_x = raw_x - mid_x;
    float origin_y = raw_y - mid_y;
    float new_x = origin_x * c - origin_y * s;
    float new_y = origin_x * s + origin_y * c;
    *x = new_x + mid_x;
    *y = new_y + mid_y;
}

/* 读取左腿ADC坐标并取负 */
static void read_left_leg_raw_coords(void)
{
    Link_Coord_t *coord = Chassis.Leg_Unit[L_Leg]->Link->info->coord;
    Leg_UI_Var.raw_A_l_x = -coord->xa * Leg_UI_Config.scale;
    Leg_UI_Var.raw_A_l_y = -coord->ya * Leg_UI_Config.scale;
    Leg_UI_Var.raw_D_l_x = -coord->xd * Leg_UI_Config.scale;
    Leg_UI_Var.raw_D_l_y = -coord->yd * Leg_UI_Config.scale;
    Leg_UI_Var.raw_C_l_x = -coord->xc * Leg_UI_Config.scale;
    Leg_UI_Var.raw_C_l_y = -coord->yc * Leg_UI_Config.scale;
}

/* 读取右腿ADC坐标并取负 */
static void read_right_leg_raw_coords(void)
{
    Link_Coord_t *coord = Chassis.Leg_Unit[R_Leg]->Link->info->coord;
    Leg_UI_Var.raw_A_r_x = -coord->xa * Leg_UI_Config.scale;
    Leg_UI_Var.raw_A_r_y = -coord->ya * Leg_UI_Config.scale;
    Leg_UI_Var.raw_D_r_x = -coord->xd * Leg_UI_Config.scale;
    Leg_UI_Var.raw_D_r_y = -coord->yd * Leg_UI_Config.scale;
    Leg_UI_Var.raw_C_r_x = -coord->xc * Leg_UI_Config.scale;
    Leg_UI_Var.raw_C_r_y = -coord->yc * Leg_UI_Config.scale;
}

/* 根据电机实例判断在线状态 */
static uint8_t is_sd_motor_online(Motor_DM_t *motor)
{
    return (motor->state->status == DEV_ONLINE) ? 1 : 0;
}

static uint8_t is_wheel_motor_online(Motor_RM_t *motor)
{
    return (motor->state->status == DEV_ONLINE) ? 1 : 0;
}

/**
 * @brief 腿部、机体UI更新
 * @author RobotPilots 2026 LYQ
 */
static void update_leg_ui(void)
{
    static float last_pitch = 0.f;
    float pitch_now = Chassis.Posture->info->pitch;

    // 读取原始坐标
    read_left_leg_raw_coords();
    read_right_leg_raw_coords();

    // 计算左腿世界坐标（绕A点旋转pitch）
    float ax_l = Leg_UI_Var.raw_A_l_x + Leg_UI_Config.leg_offset_x;
    float ay_l = Leg_UI_Var.raw_A_l_y + Leg_UI_Config.leg_offset_y;
    rotate_point_f(&Leg_UI_Var.world_A_l_x, &Leg_UI_Var.world_A_l_y,
                   Leg_UI_Var.raw_A_l_x + Leg_UI_Config.leg_offset_x,
                   Leg_UI_Var.raw_A_l_y + Leg_UI_Config.leg_offset_y,
                   ax_l, ay_l,
                   pitch_now);
    rotate_point_f(&Leg_UI_Var.world_D_l_x, &Leg_UI_Var.world_D_l_y,
                   Leg_UI_Var.raw_D_l_x + Leg_UI_Config.leg_offset_x,
                   Leg_UI_Var.raw_D_l_y + Leg_UI_Config.leg_offset_y,
                   ax_l, ay_l,
                   pitch_now);
    rotate_point_f(&Leg_UI_Var.world_C_l_x, &Leg_UI_Var.world_C_l_y,
                   Leg_UI_Var.raw_C_l_x + Leg_UI_Config.leg_offset_x,
                   Leg_UI_Var.raw_C_l_y + Leg_UI_Config.leg_offset_y,
                   ax_l, ay_l,
                   pitch_now);

    // 计算右腿世界坐标
    float ax_r = Leg_UI_Var.raw_A_r_x + Leg_UI_Config.leg_offset_x + Leg_UI_Config.right_offset_x;
    float ay_r = Leg_UI_Var.raw_A_r_y + Leg_UI_Config.leg_offset_y + Leg_UI_Config.right_offset_y;
    rotate_point_f(&Leg_UI_Var.world_A_r_x, &Leg_UI_Var.world_A_r_y,
                   Leg_UI_Var.raw_A_r_x + Leg_UI_Config.leg_offset_x + Leg_UI_Config.right_offset_x,
                   Leg_UI_Var.raw_A_r_y + Leg_UI_Config.leg_offset_y + Leg_UI_Config.right_offset_y,
                   ax_r, ay_r,
                   pitch_now);
    rotate_point_f(&Leg_UI_Var.world_D_r_x, &Leg_UI_Var.world_D_r_y,
                   Leg_UI_Var.raw_D_r_x + Leg_UI_Config.leg_offset_x + Leg_UI_Config.right_offset_x,
                   Leg_UI_Var.raw_D_r_y + Leg_UI_Config.leg_offset_y + Leg_UI_Config.right_offset_y,
                   ax_r, ay_r,
                   pitch_now);
    rotate_point_f(&Leg_UI_Var.world_C_r_x, &Leg_UI_Var.world_C_r_y,
                   Leg_UI_Var.raw_C_r_x + Leg_UI_Config.leg_offset_x + Leg_UI_Config.right_offset_x,
                   Leg_UI_Var.raw_C_r_y + Leg_UI_Config.leg_offset_y + Leg_UI_Config.right_offset_y,
                   ax_r, ay_r,
                   pitch_now);

    // 减少计算量
    float cos_pitch = cos(pitch_now);
    float sin_pitch = sin(pitch_now);

    // 计算机体杆端点（左腿）
    Leg_UI_Var.body_back_l_x = ax_l - Leg_UI_Config.body_length * cos_pitch;
    Leg_UI_Var.body_back_l_y = ay_l - Leg_UI_Config.body_length * sin_pitch;
    Leg_UI_Var.body_front_l_x = ax_l + Leg_UI_Config.body_length * cos_pitch;
    Leg_UI_Var.body_front_l_y = ay_l + Leg_UI_Config.body_length * sin_pitch;

    // 计算机体杆端点（右腿）
    Leg_UI_Var.body_back_r_x = ax_r - Leg_UI_Config.body_length * cos_pitch;
    Leg_UI_Var.body_back_r_y = ay_r - Leg_UI_Config.body_length * sin_pitch;
    Leg_UI_Var.body_front_r_x = ax_r + Leg_UI_Config.body_length * cos_pitch;
    Leg_UI_Var.body_front_r_y = ay_r + Leg_UI_Config.body_length * sin_pitch;

    // 更新左腿线段坐标
    dynamic_ui_info[L_LEG_BODY_LINE].ui_config.start_x = (uint16_t)Leg_UI_Var.body_back_l_x;
    dynamic_ui_info[L_LEG_BODY_LINE].ui_config.start_y = (uint16_t)Leg_UI_Var.body_back_l_y;
    dynamic_ui_info[L_LEG_BODY_LINE].ui_config.end_x = (uint16_t)Leg_UI_Var.body_front_l_x;
    dynamic_ui_info[L_LEG_BODY_LINE].ui_config.end_y = (uint16_t)Leg_UI_Var.body_front_l_y;

    dynamic_ui_info[L_LEG_A_TO_D].ui_config.start_x = (uint16_t)Leg_UI_Var.world_A_l_x;
    dynamic_ui_info[L_LEG_A_TO_D].ui_config.start_y = (uint16_t)Leg_UI_Var.world_A_l_y;
    dynamic_ui_info[L_LEG_A_TO_D].ui_config.end_x = (uint16_t)Leg_UI_Var.world_D_l_x;
    dynamic_ui_info[L_LEG_A_TO_D].ui_config.end_y = (uint16_t)Leg_UI_Var.world_D_l_y;

    dynamic_ui_info[L_LEG_D_TO_C].ui_config.start_x = (uint16_t)Leg_UI_Var.world_D_l_x;
    dynamic_ui_info[L_LEG_D_TO_C].ui_config.start_y = (uint16_t)Leg_UI_Var.world_D_l_y;
    dynamic_ui_info[L_LEG_D_TO_C].ui_config.end_x = (uint16_t)Leg_UI_Var.world_C_l_x;
    dynamic_ui_info[L_LEG_D_TO_C].ui_config.end_y = (uint16_t)Leg_UI_Var.world_C_l_y;

    // 更新右腿线段坐标
    dynamic_ui_info[R_LEG_BODY_LINE].ui_config.start_x = (uint16_t)Leg_UI_Var.body_back_r_x;
    dynamic_ui_info[R_LEG_BODY_LINE].ui_config.start_y = (uint16_t)Leg_UI_Var.body_back_r_y;
    dynamic_ui_info[R_LEG_BODY_LINE].ui_config.end_x = (uint16_t)Leg_UI_Var.body_front_r_x;
    dynamic_ui_info[R_LEG_BODY_LINE].ui_config.end_y = (uint16_t)Leg_UI_Var.body_front_r_y;

    dynamic_ui_info[R_LEG_A_TO_D].ui_config.start_x = (uint16_t)Leg_UI_Var.world_A_r_x;
    dynamic_ui_info[R_LEG_A_TO_D].ui_config.start_y = (uint16_t)Leg_UI_Var.world_A_r_y;
    dynamic_ui_info[R_LEG_A_TO_D].ui_config.end_x = (uint16_t)Leg_UI_Var.world_D_r_x;
    dynamic_ui_info[R_LEG_A_TO_D].ui_config.end_y = (uint16_t)Leg_UI_Var.world_D_r_y;

    dynamic_ui_info[R_LEG_D_TO_C].ui_config.start_x = (uint16_t)Leg_UI_Var.world_D_r_x;
    dynamic_ui_info[R_LEG_D_TO_C].ui_config.start_y = (uint16_t)Leg_UI_Var.world_D_r_y;
    dynamic_ui_info[R_LEG_D_TO_C].ui_config.end_x = (uint16_t)Leg_UI_Var.world_C_r_x;
    dynamic_ui_info[R_LEG_D_TO_C].ui_config.end_y = (uint16_t)Leg_UI_Var.world_C_r_y;

    // 更新左腿圆点坐标
    dynamic_ui_info[L_LEG_BODY_BACK_CIRCLE].ui_config.start_x = (uint16_t)Leg_UI_Var.body_back_l_x;
    dynamic_ui_info[L_LEG_BODY_BACK_CIRCLE].ui_config.start_y = (uint16_t)Leg_UI_Var.body_back_l_y;
    dynamic_ui_info[L_LEG_BODY_FRONT_CIRCLE].ui_config.start_x = (uint16_t)Leg_UI_Var.body_front_l_x;
    dynamic_ui_info[L_LEG_BODY_FRONT_CIRCLE].ui_config.start_y = (uint16_t)Leg_UI_Var.body_front_l_y;
    dynamic_ui_info[L_LEG_C_CIRCLE].ui_config.start_x = (uint16_t)Leg_UI_Var.world_C_l_x;
    dynamic_ui_info[L_LEG_C_CIRCLE].ui_config.start_y = (uint16_t)Leg_UI_Var.world_C_l_y;

    // 更新右腿圆点坐标
    dynamic_ui_info[R_LEG_BODY_BACK_CIRCLE].ui_config.start_x = (uint16_t)Leg_UI_Var.body_back_r_x;
    dynamic_ui_info[R_LEG_BODY_BACK_CIRCLE].ui_config.start_y = (uint16_t)Leg_UI_Var.body_back_r_y;
    dynamic_ui_info[R_LEG_BODY_FRONT_CIRCLE].ui_config.start_x = (uint16_t)Leg_UI_Var.body_front_r_x;
    dynamic_ui_info[R_LEG_BODY_FRONT_CIRCLE].ui_config.start_y = (uint16_t)Leg_UI_Var.body_front_r_y;
    dynamic_ui_info[R_LEG_C_CIRCLE].ui_config.start_x = (uint16_t)Leg_UI_Var.world_C_r_x;
    dynamic_ui_info[R_LEG_C_CIRCLE].ui_config.start_y = (uint16_t)Leg_UI_Var.world_C_r_y;

    // 更新电机在线状态颜色
    uint8_t l_front_sd_online = is_sd_motor_online(Sd_Group.motor[L_F_Sd_M]);
    uint8_t l_back_sd_online = is_sd_motor_online(Sd_Group.motor[L_B_Sd_M]);
    uint8_t l_wheel_online = is_wheel_motor_online(Wheel_Group.motor[L_WHEEL_M]);
    uint8_t r_front_sd_online = is_sd_motor_online(Sd_Group.motor[R_F_Sd_M]);
    uint8_t r_back_sd_online = is_sd_motor_online(Sd_Group.motor[R_B_Sd_M]);
    uint8_t r_wheel_online = is_wheel_motor_online(Wheel_Group.motor[R_WHEEL_M]);

    dynamic_ui_info[L_LEG_BODY_BACK_CIRCLE].ui_config.color = l_back_sd_online ? GREEN : FUCHSIA;
    dynamic_ui_info[L_LEG_BODY_FRONT_CIRCLE].ui_config.color = l_front_sd_online ? GREEN : FUCHSIA;
    dynamic_ui_info[L_LEG_C_CIRCLE].ui_config.color = l_wheel_online ? GREEN : FUCHSIA;
    dynamic_ui_info[R_LEG_BODY_BACK_CIRCLE].ui_config.color = r_back_sd_online ? GREEN : FUCHSIA;
    dynamic_ui_info[R_LEG_BODY_FRONT_CIRCLE].ui_config.color = r_front_sd_online ? GREEN : FUCHSIA;
    dynamic_ui_info[R_LEG_C_CIRCLE].ui_config.color = r_wheel_online ? GREEN : FUCHSIA;

    // 发送腿部UI
    Enqueue_Ui_For_Sending(&dynamic_ui_info[L_LEG_BODY_LINE]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[L_LEG_A_TO_D]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[L_LEG_D_TO_C]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[R_LEG_BODY_LINE]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[R_LEG_A_TO_D]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[R_LEG_D_TO_C]);

    Enqueue_Ui_For_Sending(&dynamic_ui_info[L_LEG_BODY_BACK_CIRCLE]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[L_LEG_BODY_FRONT_CIRCLE]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[L_LEG_C_CIRCLE]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[R_LEG_BODY_BACK_CIRCLE]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[R_LEG_BODY_FRONT_CIRCLE]);
    Enqueue_Ui_For_Sending(&dynamic_ui_info[R_LEG_C_CIRCLE]);
}

/**
 * @brief 超电条UI更新
 * @author RobotPilots 2026 LYQ
 */
static void update_cap_line(void)
{
    static uint16_t cap_line_last = 0;
    float cap_voltage = cap.info.cap_u;
    uint16_t cap_line = (uint16_t)(((cap_voltage * cap_voltage) / (24.f * 24.f)) * 500);

    if (cap_line_last != cap_line)
    {
        dynamic_ui_info[CAP_LINE].ui_config.end_x = (Client_mid_position_x - 250) + cap_line;

        float ratio = (cap_voltage * cap_voltage) / (24.f * 24.f);
        if (ratio <= 0.3f)
        {
            dynamic_ui_info[CAP_LINE].ui_config.color = FUCHSIA;
        }
        else if (ratio <= 0.7f)
        {
            dynamic_ui_info[CAP_LINE].ui_config.color = ORANGE;
        }
        else
        {
            dynamic_ui_info[CAP_LINE].ui_config.color = GREEN;
        }
        Enqueue_Ui_For_Sending(&dynamic_ui_info[CAP_LINE]);
    }
    cap_line_last = cap_line;
}

/**
 * @brief 底盘方位UI更新
 * @author RobotPilots 2026 LYQ
 */
static void update_chas_circle(void)
{
    static float angle_last = 0.f;
    float angle_now = -(gimbal.base_info.yaw_motor_angle);
    if (my_abs(angle_now) > PI)
    {
        angle_now -= sgn(angle_now) * 2 * PI;
    }

    if (my_abs(angle_now - angle_last) > 0.001f)
    {
        float end_x = (float)dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_x;
        float end_y = (float)dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_y;
        rotate_point_f(&end_x, &end_y,
                       CHAS_CIRCLE_X, CHAS_CIRCLE_Y + CHAS_CIRCLE_R,
                       CHAS_CIRCLE_X, CHAS_CIRCLE_Y, angle_now);
        dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_x = (uint16_t)end_x;
        dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_y = (uint16_t)end_y;

        float start_x = (float)dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_x;
        float start_y = (float)dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_y;
        float end_x2 = (float)dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_x;
        float end_y2 = (float)dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_y;
        rotate_point_f(&start_x, &start_y,
                       CHAS_CIRCLE_X - CHAS_CIRCLE_R, CHAS_CIRCLE_Y,
                       CHAS_CIRCLE_X, CHAS_CIRCLE_Y, angle_now);
        rotate_point_f(&end_x2, &end_y2,
                       CHAS_CIRCLE_X + CHAS_CIRCLE_R, CHAS_CIRCLE_Y,
                       CHAS_CIRCLE_X, CHAS_CIRCLE_Y, angle_now);
        dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_x = (uint16_t)start_x;
        dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_y = (uint16_t)start_y;
        dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_x = (uint16_t)end_x2;
        dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_y = (uint16_t)end_y2;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[CHAS_HEAD_LINE]);
        Enqueue_Ui_For_Sending(&dynamic_ui_info[CHAS_SIDE_LINE]);
    }
    angle_last = angle_now;
}

/**
 * @brief 自瞄框UI更新
 * @author RobotPilots 2026 LYQ
 */
static void update_auto_catch_frame(void)
{
    uint8_t vision_offline = !Board_Rx_Info.flag.is_vision_online;
    uint8_t target_found = Board_Rx_Info.flag.is_find_target && Board_Rx_Info.flag.hit_enable;

    if (vision_offline)
    {
        dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = BLACK;
    }
    else if (target_found)
    {
        dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = PINK;
    }
    else
    {
        dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = WHITE;
    }
    Enqueue_Ui_For_Sending(&dynamic_ui_info[AUTO_CATCH_FRAME]);
}

/**
 * @brief 腿长模式框UI更新
 * @author RobotPilots 2026 LYQ
 */
static void update_length_frame(void)
{
    static uint8_t last_mode = 1;
    uint8_t now_mode = 1;
    if (Balance.Flag->KNEE_STRIKE_Flag)
        now_mode = 3;
    else if (Balance.Flag->Middle_Flag)
        now_mode = 2;

    if (last_mode != now_mode)
    {
        if (now_mode == 1)
        {
            dynamic_ui_info[LENGTH_FRAME].ui_config.start_y = Client_mid_position_y - 125;
            dynamic_ui_info[LENGTH_FRAME].ui_config.end_y = Client_mid_position_y - 165;
        }
        else if (now_mode == 2)
        {
            dynamic_ui_info[LENGTH_FRAME].ui_config.start_y = Client_mid_position_y - 65;
            dynamic_ui_info[LENGTH_FRAME].ui_config.end_y = Client_mid_position_y - 105;
        }
        else if (now_mode == 3)
        {
            dynamic_ui_info[LENGTH_FRAME].ui_config.start_y = Client_mid_position_y - 5;
            dynamic_ui_info[LENGTH_FRAME].ui_config.end_y = Client_mid_position_y - 45;
        }
        Enqueue_Ui_For_Sending(&dynamic_ui_info[LENGTH_FRAME]);
    }
    last_mode = now_mode;
}

/**
 * @brief 车模式UI更新
 * @author RobotPilots 2026 LYQ
 */
static void update_car_mode(void)
{
    static uint8_t last_car_mode = 255;
    uint8_t car_mode = Balance.mode;

    if (last_car_mode != car_mode)
    {
        memset(dynamic_ui_info[D_CAR_MODE].ui_config.text, 0, sizeof(dynamic_ui_info[D_CAR_MODE].ui_config.text));
        switch (car_mode)
        {
        case Sleep_Mode:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "SLEEP");
            break;
        case Init_Mode:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "INIT");
            break;
        case Imu_Mode:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "IMU");
            break;
        case SitDown_Mode:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "SIT DOWN");
            break;
        case Mec_Mode:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "MEC");
            break;
        case Cycle_Mode:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "CYCLE");
            break;
        case Rescue_Mode:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "RESCUE");
            break;
        case Manual_Rescue_Mode:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "HELP YOURSELF");
            break;
        case LEG_TEST_Mode:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "TEST");
            break;
        default:
            strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "UNKNOWN");
            break;
        }
        Enqueue_Ui_For_Sending(&dynamic_ui_info[D_CAR_MODE]);
    }
    last_car_mode = car_mode;
}

/**
 * @brief 机器人血量UI更新
 * @author RobotPilots 2026 LYQ
 */
void update_robot_health(void)
{
    static uint32_t last_health_update_timestamp;
    static uint8_t health_value_color; // 0白1绿
    uint32_t health_update_timestamp = My_Judge.org_info->radio_information_data.health.update_timestamp;

    if (My_Judge.info->my_color == 0) // 红色
    {
        if (health_update_timestamp != last_health_update_timestamp)
        {
            sprintf(dynamic_ui_info[D_BLUE_1_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.hero_health);
            sprintf(dynamic_ui_info[D_BLUE_2_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.engineer_health);
            sprintf(dynamic_ui_info[D_BLUE_3_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.infantry1_health);
            sprintf(dynamic_ui_info[D_BLUE_4_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.infantry2_health);
            sprintf(dynamic_ui_info[D_BLUE_5_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.sentry_health);

            if (health_value_color == 0)
            {
                dynamic_ui_info[D_BLUE_1_HEALTH_CHAR].ui_config.color = WHITE;
                dynamic_ui_info[D_BLUE_2_HEALTH_CHAR].ui_config.color = WHITE;
                dynamic_ui_info[D_BLUE_3_HEALTH_CHAR].ui_config.color = WHITE;
                dynamic_ui_info[D_BLUE_4_HEALTH_CHAR].ui_config.color = WHITE;
                dynamic_ui_info[D_BLUE_5_HEALTH_CHAR].ui_config.color = WHITE;
                health_value_color = !health_value_color;
            }
            else
            {
                dynamic_ui_info[D_BLUE_1_HEALTH_CHAR].ui_config.color = GREEN;
                dynamic_ui_info[D_BLUE_2_HEALTH_CHAR].ui_config.color = GREEN;
                dynamic_ui_info[D_BLUE_3_HEALTH_CHAR].ui_config.color = GREEN;
                dynamic_ui_info[D_BLUE_4_HEALTH_CHAR].ui_config.color = GREEN;
                dynamic_ui_info[D_BLUE_5_HEALTH_CHAR].ui_config.color = GREEN;
                health_value_color = !health_value_color;
            }

            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_BLUE_1_HEALTH_CHAR]);
            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_BLUE_2_HEALTH_CHAR]);
            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_BLUE_3_HEALTH_CHAR]);
            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_BLUE_4_HEALTH_CHAR]);
            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_BLUE_5_HEALTH_CHAR]);
        }
    }
    else // 蓝色
    {
        if (health_update_timestamp != last_health_update_timestamp)
        {
            sprintf(dynamic_ui_info[D_RED_1_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.hero_health);
            sprintf(dynamic_ui_info[D_RED_2_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.engineer_health);
            sprintf(dynamic_ui_info[D_RED_3_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.infantry1_health);
            sprintf(dynamic_ui_info[D_RED_4_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.infantry2_health);
            sprintf(dynamic_ui_info[D_RED_5_HEALTH_CHAR].ui_config.text, "%d", My_Judge.org_info->radio_information_data.health.sentry_health);
            if (health_value_color == 0)
            {
                dynamic_ui_info[D_RED_1_HEALTH_CHAR].ui_config.color = WHITE;
                dynamic_ui_info[D_RED_2_HEALTH_CHAR].ui_config.color = WHITE;
                dynamic_ui_info[D_RED_3_HEALTH_CHAR].ui_config.color = WHITE;
                dynamic_ui_info[D_RED_4_HEALTH_CHAR].ui_config.color = WHITE;
                dynamic_ui_info[D_RED_5_HEALTH_CHAR].ui_config.color = WHITE;
                health_value_color = !health_value_color;
            }
            else
            {
                dynamic_ui_info[D_RED_1_HEALTH_CHAR].ui_config.color = GREEN;
                dynamic_ui_info[D_RED_2_HEALTH_CHAR].ui_config.color = GREEN;
                dynamic_ui_info[D_RED_3_HEALTH_CHAR].ui_config.color = GREEN;
                dynamic_ui_info[D_RED_4_HEALTH_CHAR].ui_config.color = GREEN;
                dynamic_ui_info[D_RED_5_HEALTH_CHAR].ui_config.color = GREEN;
                health_value_color = !health_value_color;
            }
            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_RED_1_HEALTH_CHAR]);
            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_RED_2_HEALTH_CHAR]);
            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_RED_3_HEALTH_CHAR]);
            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_RED_4_HEALTH_CHAR]);
            Enqueue_Ui_For_Sending(&dynamic_ui_info[D_RED_5_HEALTH_CHAR]);
        }
    }

    last_health_update_timestamp = health_update_timestamp;
}

/**
 * @brief 飞镖预警UI更新
 * @author RobotPilots 2026 LYQ
 */
void update_dart_warning(void)
{
    static uint32_t last_dart_warning_timestamp;
    static uint8_t dart_warning_color; // 0白1粉红
    static uint8_t last_dart_state;
    static uint32_t dart_warning_enable_tick; // 上升沿触发时的时间戳
    uint32_t current_tick = HAL_GetTick();
    uint8_t current_dart_state = My_Judge.org_info->radio_dart_state_data.state;
    uint8_t need_send = 0;

    // 检测上升沿跳变（0→1）
    if (last_dart_state == 0 && current_dart_state == 1)
    {
        dart_warning_enable_tick = current_tick; // 记录触发时间
    }
    last_dart_state = current_dart_state;

    // 20s超时判断，超过20s则关闭UI
    if (current_tick - dart_warning_enable_tick > 20000)
    {
        dynamic_ui_info[DART_WARNING_CHAR].ui_config.start_x = 0;
        dynamic_ui_info[DART_WARNING_CHAR].ui_config.start_y = 0;
        return;
    }

    if (current_dart_state == 1)
    { // 飞镖舱门开启
        if (current_tick - last_dart_warning_timestamp >= 1000)
        { // 每1000ms切换颜色
            last_dart_warning_timestamp = current_tick;
            uint8_t new_color = !dart_warning_color;
            if (new_color != dart_warning_color)
            {
                dart_warning_color = new_color;
                need_send = 1;
            }
        }
        dynamic_ui_info[DART_WARNING_CHAR].ui_config.start_x = DART_WARNING_X;
        dynamic_ui_info[DART_WARNING_CHAR].ui_config.start_y = DART_WARNING_Y;
        dynamic_ui_info[DART_WARNING_CHAR].ui_config.color = (dart_warning_color == 0) ? BLACK : PINK;
    }
    else
    {
        dynamic_ui_info[DART_WARNING_CHAR].ui_config.start_x = 0;
        dynamic_ui_info[DART_WARNING_CHAR].ui_config.start_y = 0;
    }

    if (need_send)
    {
        Enqueue_Ui_For_Sending(&dynamic_ui_info[DART_WARNING_CHAR]);
    }
}

void My_Ui_Init(void)
{
    Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info) / sizeof(ui_info_t),
                 const_ui_info, sizeof(const_ui_info) / sizeof(ui_info_t));
}

/**
 * @brief 摩擦轮状态灯UI更新
 * @author RobotPilots 2026 LYQ
 * @note  离线=黑色, 在线不转=青色, 在线旋转=绿色; 仅颜色变化时Enqueue_Ui_For_Sending
 */
void update_fric_state_cycles(void)
{
    static graphic_color_e last_color[2] = {0}; // [0]=左轮, [1]=右轮
    uint8_t L_online = shoot.extern_input.fric.L_online;
    uint8_t R_online = shoot.extern_input.fric.R_online;
    uint8_t L_spinning = (shoot.adapt_info.final_fric_target_speed > 0);
    uint8_t R_spinning = L_spinning;

    // 左摩擦轮
    graphic_color_e L_color = (!L_online) ? BLACK : (L_spinning ? GREEN : CYAN_BLUE);
    if (L_color != last_color[0])
    {
        last_color[0] = L_color;
        dynamic_ui_info[D_L_FRIC_STATE_CYCLE].ui_config.color = L_color;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[D_L_FRIC_STATE_CYCLE]);
    }

    // 右摩擦轮
    graphic_color_e R_color = (!R_online) ? BLACK : (R_spinning ? GREEN : CYAN_BLUE);
    if (R_color != last_color[1])
    {
        last_color[1] = R_color;
        dynamic_ui_info[D_R_FRIC_STATE_CYCLE].ui_config.color = R_color;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[D_R_FRIC_STATE_CYCLE]);
    }
}

/**
 * @brief 视觉检测到的敌方机器人血量UI更新
 * @author RobotPilots 2026 LYQ
 * @note  detect_num 0-4显示对应血量，否则隐藏; update_timestamp变化时颜色在白红间跳变
 */
void update_vision_detect_robot_health(void)
{
    static uint32_t last_health_timestamp = 0;
    static uint8_t last_detect_num = 15;
    static uint8_t last_health_value = 0;
    static uint8_t last_valid_target = 0;
    static graphic_color_e last_color = WHITE;
    uint8_t detect_num = Board_Rx_Info.flag.vision_detect_num;
    uint32_t current_timestamp = My_Judge.org_info->radio_information_data.health.update_timestamp;

    // 根据detect_num获取对应血量
    uint16_t health_value = 0;
    uint8_t valid_target = 0;
    if (Board_Rx_Info.flag.is_find_target == 1)
    {
        if (detect_num <= 4)
        {
            switch (detect_num)
            {
            case 0:
                health_value = My_Judge.org_info->radio_information_data.health.sentry_health;
                break;
            case 1:
                health_value = My_Judge.org_info->radio_information_data.health.hero_health;
                break;
            case 2:
                health_value = My_Judge.org_info->radio_information_data.health.engineer_health;
                break;
            case 3:
                health_value = My_Judge.org_info->radio_information_data.health.infantry1_health;
                break;
            case 4:
                health_value = My_Judge.org_info->radio_information_data.health.infantry2_health;
                break;
            default:
                valid_target = 0;
                break;
            }
            valid_target = 1;
        }
    }
    else
    {
        valid_target = 0;
    }

    // 判断是否有变化需要发送
    uint8_t need_send = 0;
    if (detect_num != last_detect_num)
    {
        need_send = 1;
        last_detect_num = detect_num;
    }
    if (health_value != last_health_value)
    {
        need_send = 1;
        last_health_value = health_value;
    }
    if (current_timestamp != last_health_timestamp)
    {
        last_health_timestamp = current_timestamp;
        last_color = (last_color == WHITE) ? PINK : WHITE; // 白红跳变
        need_send = 1;
    }
    if (valid_target != last_valid_target)
    {
        last_valid_target = valid_target;
        need_send = 1;
    }

    if (!need_send)
    {
        return;
    }

    // 设置位置和数值
    if (valid_target)
    {
        dynamic_ui_info[D_VISION_DETECT_ROBOT_HEALTH_INT].ui_config.start_x = 935;
        dynamic_ui_info[D_VISION_DETECT_ROBOT_HEALTH_INT].ui_config.start_y = 650;
        dynamic_ui_info[D_VISION_DETECT_ROBOT_HEALTH_INT].ui_config.int_num = health_value;
    }
    else
    {
        dynamic_ui_info[D_VISION_DETECT_ROBOT_HEALTH_INT].ui_config.start_x = 0;
        dynamic_ui_info[D_VISION_DETECT_ROBOT_HEALTH_INT].ui_config.start_y = 0;
    }
    dynamic_ui_info[D_VISION_DETECT_ROBOT_HEALTH_INT].ui_config.color = last_color;
    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_DETECT_ROBOT_HEALTH_INT]);
}

void update_enermy_money(void)
{
    static uint32_t last_update_timestamp = 0;
    static uint16_t last_coins = 65535;

    uint32_t current_timestamp = My_Judge.org_info->radio_information_data.status.update_timestamp;
    uint16_t current_coins = My_Judge.org_info->radio_information_data.status.coins_left;

    if (current_timestamp != last_update_timestamp && current_coins != last_coins)
    {
        static uint8_t money_color = 0;
        if (money_color == 0)
        {
            dynamic_ui_info[D_ENERMY_MONEY_INT].ui_config.color = WHITE;
            money_color = 1;
        }
        else
        {
            dynamic_ui_info[D_ENERMY_MONEY_INT].ui_config.color = ORANGE;
            money_color = 0;
        }

        dynamic_ui_info[D_ENERMY_MONEY_INT].ui_config.int_num = current_coins;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ENERMY_MONEY_INT]);

        last_update_timestamp = current_timestamp;
        last_coins = current_coins;
    }
}

void Ui_Info_Update(void)
{
    client_info_update();
    update_cap_line();
    update_chas_circle();
    update_auto_catch_frame();
    update_length_frame();
    update_car_mode();
    update_leg_ui();
    update_robot_health();
    update_dart_warning();
    update_fric_state_cycles();
    update_vision_detect_robot_health();
    update_enermy_money();
}