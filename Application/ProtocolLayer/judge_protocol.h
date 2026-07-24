/**

  ******************************************************************************

  * @file           : judge_potocol.c\h

    * @author         : czf

    * @date           : 2022-4-22 14:39:50

  * @brief          :

    * @history        :

  ******************************************************************************

  */

#ifndef __JUDGE_POTOCOL_H

#define __JUDGE_POTOCOL_H

#include "stm32h7xx_hal.h"

/* 帧头 */

typedef struct

{

    uint8_t SOF; // 数据帧起始字节，固定值为 0xA5

    uint16_t data_length; // 数据帧中 data 的长度

    uint8_t seq; // 包序号

    uint8_t CRC8; // 帧头 CRC8 校验

} judge_frame_header_t;

typedef struct

{

    judge_frame_header_t *frame_header;

    uint16_t cmd_id;

    uint16_t frame_tail;

} drv_judge_info_t;

extern drv_judge_info_t drv_judge_info;

void judge_recive(uint8_t *rxBuf);

/********************from裁判系统串口协议附录V1.3的数据结构体********************/

/* 比赛状态数据：0x0001。发送频率：1Hz */

typedef struct

{

    uint8_t game_type : 4;

    uint8_t game_progress : 4;

    uint16_t stage_remain_time;

    uint64_t SyncTimeStamp;

} ext_game_status_t;

/* 比赛结果数据：0x0002。发送频率：比赛结束后发送 */

typedef struct

{

    uint8_t winner;

} ext_game_result_t;

/* 机器人血量数据：0x0003。发送频率：1Hz */

typedef struct

{

    uint16_t ally_1_robot_HP;

    uint16_t ally_2_robot_HP;

    uint16_t ally_3_robot_HP;

    uint16_t ally_4_robot_HP;

    uint16_t reserved;

    uint16_t ally_7_robot_HP;

    uint16_t ally_outpost_HP;

    uint16_t ally_base_HP;

} ext_game_robot_HP_t;

/* 场地事件数据：0x0101。发送频率：1Hz */

typedef struct __attribute__((packed))

{

    uint32_t event_data;

} ext_event_data_t;

/* 补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人 */

// typedef struct __attribute__((packed))

//{

//	uint8_t supply_projectile_id;

//	uint8_t supply_robot_id;

//	uint8_t supply_projectile_step;

//	uint8_t supply_projectile_num;

//} ext_supply_projectile_action_t;

/* 裁判警告信息：cmd_id (0x0104)。发送频率：己方警告发生后发送 */

typedef struct __attribute__((packed))

{

    uint8_t level;

    uint8_t offending_robot_id;

    uint8_t count;

} ext_referee_warning_t;

/* 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人 */

typedef struct __attribute__((packed))

{

    uint8_t dart_remaining_time;

    uint16_t dart_info;

} ext_dart_info_t;

///* 比赛机器人状态：0x0201。发送频率：10Hz */

// typedef struct __attribute__((packed))

//{

//	uint8_t robot_id;

//	uint8_t robot_level;

//	uint16_t remain_HP;

//	uint16_t max_HP;

//	uint16_t shooter_id1_17mm_cooling_rate;

//	uint16_t shooter_id1_17mm_cooling_limit;

//	uint16_t shooter_id1_17mm_speed_limit;

//	uint16_t shooter_id2_17mm_cooling_rate;

//	uint16_t shooter_id2_17mm_cooling_limit;

//	uint16_t shooter_id2_17mm_speed_limit;

//	uint16_t shooter_id1_42mm_cooling_rate;

//	uint16_t shooter_id1_42mm_cooling_limit;

//	uint16_t shooter_id1_42mm_speed_limit;

//	uint16_t chassis_power_limit;

//	uint8_t mains_power_gimbal_output : 1;

//	uint8_t mains_power_chassis_output : 1;

//	uint8_t mains_power_shooter_output : 1;

//} ext_game_robot_status_t;

/* 比赛机器人状态：0x0201。发送频率：10Hz */

typedef struct __attribute__((packed))

{

    uint8_t robot_id;

    uint8_t robot_level;

    uint16_t current_HP;

    uint16_t maximum_HP;

    uint16_t shooter_barrel_cooling_value;

    uint16_t shooter_barrel_heat_limit;

    uint16_t chassis_power_limit;

    uint8_t power_management_gimbal_output : 1;

    uint8_t power_management_chassis_output : 1;

    uint8_t power_management_shooter_output : 1;

} ext_game_robot_status_t;

/* 实时功率热量数据：0x0202。发送频率：50Hz */

typedef struct __attribute__((packed))

{

    uint16_t reserved;

    uint16_t reserved_1;

    float reserved_2;

    uint16_t buffer_energy;

    uint16_t shooter_17mm_barrel_heat;

    uint16_t shooter_42mm_barrel_heat;

} ext_power_heat_data_t;

/* 机器人位置：0x0203。发送频率：10Hz */

typedef struct __attribute__((packed))

{

    float x;

    float y;

    float yaw;

} ext_game_robot_pos_t;

/* 机器人增益：0x0204。发送频率：1Hz */

typedef struct __attribute__((packed))

{

    uint8_t recovery_buff;

    uint16_t cooling_buff;

    uint8_t defence_buff;

    uint8_t vulnerability_buff;

    uint16_t attack_buff;

    uint8_t remaining_energy;

} ext_buff_t;

/* 空中机器人能量状态：0x0205。发送频率：10Hz */

typedef struct __attribute__((packed))

{

    uint8_t attack_time;

} aerial_robot_energy_t;

/* 伤害状态：0x0206。发送频率：伤害发生后发送 */

typedef struct __attribute__((packed))

{

    uint8_t armor_id : 4;

    uint8_t HP_deduction_reason : 4;

} ext_robot_hurt_t;

/* 实时射击信息：0x0207。发送频率：射击后发送 */

typedef struct __attribute__((packed))

{

    uint8_t bullet_type;

    uint8_t shooter_number;

    uint8_t bullet_freq; // Hz

    float initial_speed;

} ext_shoot_data_t;

/* 子弹剩余发射数：0x0208。发送频率：10Hz 周期发送，所有机器人发送 */

typedef struct __attribute__((packed))

{

    uint16_t projectile_allowance_17mm;

    uint16_t projectile_allowance_42mm;

    uint16_t remaining_gold_coin;

    uint16_t projectile_allowance_fortress;

} ext_projectile_allowance_t;

/* 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人 */

typedef struct __attribute__((packed))

{

    uint32_t rfid_status;

    uint8_t rfid_status_2;

} ext_rfid_status_t;

/* 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人 */

typedef struct __attribute__((packed))

{

    uint8_t dart_launch_opening_status;

    uint8_t reserved;

    uint16_t target_change_time;

    uint16_t latest_launch_cmd_time;

} ext_dart_client_cmd_t;

/*机器人交互数据，发送方触发发送*/
typedef struct __attribute__((packed)) {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[112]; // 内容数据段最大112

} robot_interaction_data_t;

// 子结构体

typedef struct __attribute__((packed)) {
    uint8_t heal;
    uint16_t cooldown;
    uint8_t defense;
    uint8_t vulnerable;
    uint16_t attack;
} basic_buff_t;

typedef struct __attribute__((packed)) {
    uint8_t heal;
    uint16_t cooldown;
    uint8_t defense;
    uint8_t vulnerable;
    uint16_t attack;
    uint8_t form;
} sentry_buff_t;

typedef struct __attribute__((packed)) {
    uint16_t hero_health;
    uint16_t engineer_health;
    uint16_t infantry1_health;
    uint16_t infantry2_health;
    uint16_t sentry_health;
    uint32_t update_timestamp;
} health_data_t;

typedef struct __attribute__((packed)) {
    uint16_t hero_allow_shoot;
    uint16_t infantry1_allow_shoot;
    uint16_t infantry2_allow_shoot;
    uint16_t drone_allow_shoot;
    uint16_t sentry_allow_shoot;
    uint32_t update_timestamp;
} allow_shoot_data_t;

typedef struct __attribute__((packed)) {
    uint16_t coins_left;
    uint16_t coins_total;
    uint32_t capture_status;
    uint32_t update_timestamp;
} status_data_t;

typedef struct __attribute__((packed)) {
    basic_buff_t hero;
    basic_buff_t engineer;
    basic_buff_t infantry1;
    basic_buff_t infantry2;
    sentry_buff_t sentry;
    uint32_t update_timestamp;
} buff_data_t;

#define Radio_information_data_id 0x211
/*雷达无线电交互数据 data_id:0x211*/
typedef struct __attribute__((packed)) {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    health_data_t health;
    allow_shoot_data_t allow_shoot;
    status_data_t status;
    buff_data_t buff;
    uint32_t current_timestamp;
} radio_information_data_t;

#define Radio_dart_state_data_id 0x0201
/* 雷达检测飞镖舱门开启数据 data_id:0x0201 */
typedef struct __attribute__((packed)) {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t state; // 0关1开
} dart_state_data_t;

/********************命令码枚举********************/

enum

{

    ID_game_state = 0x0001, // 比赛状态数据，1Hz

    ID_game_result = 0x0002, // 比赛结果数据，比赛结束发送

    ID_game_robot_HP = 0x0003, // 比赛机器人血量数据，1Hz发送

    ID_dart_status = 0x0004, // 飞镖发射状态，飞镖发射时发送

    ID_ICRA_buff_debuff_zone_status = 0x0005, // 人工智能挑战赛加成与惩罚区状态，1Hz

    ID_event_data = 0x0101, // 场地事件数据，1Hz

    ID_supply_projectile_action = 0x0102, // 场地补给站动作标识数据

    ID_referee_warning = 0x0104, // 裁判警告数据，警告后发送

    ID_dart_remaining_time = 0x0105, // 飞镖发射口倒计时，1Hz

    ID_game_robot_state = 0x0201, // 机器人状态数据，10Hz

    ID_power_heat_data = 0x0202, // 实时功率热量数据，50Hz

    ID_game_robot_pos = 0x0203, // 机器人位置数据，10Hz

    ID_buff_musk = 0x0204, // 机器人增益数据和底盘能量，3Hz

    ID_aerial_robot_energy = 0x0205, // 空中机器人能量状态数据，10Hz，只有空中机器人主控发送

    ID_robot_hurt = 0x0206, // 伤害状态数据，伤害发生后发送

    ID_shoot_data = 0x0207, // 实时射击数据，子弹发射后发送

    ID_projectile_allowance = 0x0208, // 允许发弹量，固定以 10Hz 频率发

    ID_rfid_status = 0x0209, // 机器人RFID状态，3Hz

    ID_interactive_header_data = 0x0301, // 机器人交互数据，发送方触发发送

    ID_Custom_client = 0x0310, // 机器人发送给自定义客户端的数据，频率上限为 50Hz

    ID_map = 0x0305 /// 手端小地图接收雷达数据，频率上限为 5Hz
};

/********************长度枚举********************/

enum

{

    LEN_FRAME_HEAD = 5, // 帧头长度

    LEN_CMD_ID = 2, // 命令码长度

    LEN_FRAME_TAIL = 2, // 帧尾CRC16

    LEN_game_state = 11, // 0x0001,//比赛状态数据，1Hz

    LEN_game_result = 1, // 0x0002,//比赛结果数据，比赛结束发送

    LEN_game_robot_HP = 16, // 0x0003,//比赛机器人血量数据，1Hz发送

    LEN_dart_status = 3, // 0x0004,//飞镖发射状态，飞镖发射时发送

    LEN_ICRA_buff_debuff_zone_status = 3, // 0x0005,//人工智能挑战赛加成与惩罚区状态，1Hz

    LEN_event_data = 4, // 0x0101,//场地事件数据，1Hz

    LEN_supply_projectile_action = 4, // 0x0102,//场地补给站动作标识数据

    LEN_referee_warning = 3, // 0x0104,//裁判警告数据，警告后发送

    LEN_dart_remaining_time = 3, // 0x0105,//飞镖发射口倒计时，1Hz

    LEN_game_robot_state = 13, // 0x0201,//机器人状态数据，10Hz

    LEN_power_heat_data = 14, // 0x0202,//实时功率热量数据，50Hz

    LEN_game_robot_pos = 16, // 0x0203,//机器人位置数据，10Hz

    LEN_buff_musk = 8, // 0x0204,//机器人增益数据，1Hz

    LEN_aerial_robot_energy = 3, // 0x0205,//空中机器人能量状态数据，10Hz，只有空中机器人主控发送

    LEN_robot_hurt = 1, // 0x0206,//伤害状态数据，伤害发生后发送

    LEN_shoot_data = 7, // 0x0207,//实时射击数据，子弹发射后发送

    LEN_bullet_remaining = 6, // 0x0208,//弹丸剩余发送数，仅空中机器人，哨兵机器人以及ICRA机器人发送，1Hz

    LEN_rfid_status = 5, // 0x0209,//机器人RFID状态，1Hz

    LEN_interactive_header_data = 6, // 0x0301 //机器人交互数据，数据头长度

    LEN_radio_information_data = 90, // 无线电解调信息长度

    LEN_radio_dart_state_data = 7, // 雷达识别飞镖闸门开启信息长度

    LEN_map = 24 // 0x0305 //选手端小地图接收雷达数据，频率上限为 5Hz
};

#endif
