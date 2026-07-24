#ifndef __BUZZER_H
#define __BUZZER_H

#include "tim.h"
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum buzzer_sound_state_e {
    BUZZER_OFF = 0,
    BUZZER_NORMAL_ON,
    BUZZER_ALARM,
    BUZZER_START_SOUND
} buzzer_sound_state_e;

typedef struct buzzer_config_struct {
    TIM_HandleTypeDef *tim;
    uint32_t channel;

    uint32_t max_tim_arr;
    uint32_t tim_freq;

    float min_pwm_duty;
    float max_pwm_duty;
    float min_volume;
    float max_volume;
} buzzer_config_t;

// ����-����-ʱ���ṹ�壺��ÿ��������Ƶ�ʺͲ���ʱ��
typedef struct note_duration {
    float volume;      /* ���� */
    float freq;        /* ����Ƶ�ʣ�Hz�� */
    uint32_t duration; /* ����ʱ�������룩 */
} note_duration_t;

/* Ŀ��ṹ�� */
typedef struct buzzer_input_info_struct {
    float volume;
    float freq;
} buzzer_input_info_t;

typedef struct buzzer_status_struct {
    buzzer_input_info_t input_info;
    uint16_t tim_presc;
    float duty;
    uint16_t CCR;
    float ARR_raw;
    uint16_t ARR;
} buzzer_base_info_t;

typedef struct buzzer_struct {
    buzzer_config_t config;
    buzzer_base_info_t base_info;
    buzzer_sound_state_e sound_state;
    uint32_t sound_start_tick;

    void (*init)(struct buzzer_struct *buz_str);
    void (*work)(struct buzzer_struct *buz_str);
} buzzer_t;

/* Exported function --------------------------------------------------------*/
void Buzzer_Normal_On(void);
void Buzzer_Normal_Off(void);
void Buzzer_Alarm(void);
void Buzzer_StartSound(void);

/* Exported variables --------------------------------------------------------*/
extern buzzer_t buzzer;

#endif
