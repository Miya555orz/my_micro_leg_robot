#include "buzzer.h"

#define constrain(x, min, max) ((x > max) ? max : (x < min ? min : x))

void Buzzer_init(buzzer_t *buzzer);
void Buzzer_Work(buzzer_t *buzzer);
buzzer_t buzzer = {

    .work = Buzzer_Work,
    .init = Buzzer_init,
    .sound_state = BUZZER_OFF,
    .sound_start_tick = 0,
};
/* Private macro -------------------------------------------------------------*/

/**
 * @brief 初始化参数
 */
void Buzzer_init(buzzer_t *buzzer)
{
    buzzer->config.tim = &htim12;
    buzzer->config.channel = TIM_CHANNEL_2;

    buzzer->config.max_tim_arr = 65535;
    buzzer->config.tim_freq = 240000000;
    buzzer->config.min_pwm_duty = 0;
    buzzer->config.max_pwm_duty = 0.8;
    buzzer->config.max_volume = 100.f;
    buzzer->config.min_volume = 0.f;

    // 启动 PWM 输出，初始占空比为 0（不发声）
    HAL_TIM_PWM_Start(buzzer->config.tim, buzzer->config.channel);
    __HAL_TIM_SET_COMPARE(buzzer->config.tim, buzzer->config.channel, 0);
}

/**
 * @brief 动态计算最佳分频系数（确保 ARR ≤ max_tim_arr）
 * @return ARR（0~65535）
 */
static uint16_t Buzzer_Calc_Optimal_Presc(buzzer_t *buzzer, float buzzer_freq)
{
    if (buzzer_freq <= 0)
    {
        return 0; // 频率非法，返回默认分频系数
    }

    uint32_t tim_freq = buzzer->config.tim_freq;
    uint16_t max_arr = buzzer->config.max_tim_arr;

    // 公式推导：tim_presc = (tim_freq / (buzzer_freq * (max_arr + 1))) - 1
    // 此处向上取整，将"-1"舍去，相当于分频增加
    float presc_float = (tim_freq / (buzzer_freq * (max_arr + 1)));

    // 限制分频系数范围（PSC 是 16 位寄存器，0~65535）
    uint16_t presc = (uint16_t)constrain(presc_float, 0.0f, 65535.0f);

    return presc;
}

/**
 * @brief 输入频率计算ARR
 */
static uint16_t Buzzer_Calc_ARR(buzzer_t *buzzer)
{
    if (buzzer->base_info.input_info.freq <= 0)
    {
        return 0; // 频率非法，返回0
    }

    // 计算原始ARR值
    float arr_float = buzzer->config.tim_freq /
                          (buzzer->base_info.input_info.freq) /
                          (buzzer->base_info.tim_presc + 1.f) -
                      1.f;
    buzzer->base_info.ARR_raw = arr_float;
    // 限制ARR范围，避免异常值
    return (uint16_t)constrain(arr_float, 0.0f, (float)buzzer->config.max_tim_arr);
}

/**
 * @brief 输入占空比、ARR计算CCR
 */
static uint16_t Buzzer_Calc_CCR(buzzer_t *buzzer)
{
    return (uint16_t)constrain(
        buzzer->base_info.duty * buzzer->base_info.ARR,
        0.0f, (float)buzzer->base_info.ARR);
}

/**
 * @brief 从音量volume转换到占空比duty
 */
static float Buzzer_Volume_to_Duty(buzzer_t *buzzer, float volume)
{
    volume = constrain(volume, buzzer->config.min_volume, buzzer->config.max_volume);
    return (buzzer->config.max_pwm_duty - buzzer->config.min_pwm_duty) /
               (buzzer->config.max_volume - buzzer->config.min_volume) * volume +
           buzzer->config.min_pwm_duty;
}

/**
 * @brief Buzzer给arr、ccr负值
 */
void Buzzer_Work(buzzer_t *buzzer)
{
    uint32_t elapsed = HAL_GetTick() - buzzer->sound_start_tick;

    switch (buzzer->sound_state)
    {
    case BUZZER_OFF:
        buzzer->base_info.input_info.freq = 0.f;
        buzzer->base_info.input_info.volume = 0.f;
        break;

    case BUZZER_NORMAL_ON:
        buzzer->base_info.input_info.freq = 2000.f;
        buzzer->base_info.input_info.volume = 80.f;
        break;

    case BUZZER_ALARM:
        if (elapsed >= 1000U)
        {
            buzzer->sound_state = BUZZER_OFF;
            buzzer->base_info.input_info.freq = 0.f;
            buzzer->base_info.input_info.volume = 0.f;
        }
        else
        {
            float ratio = (float)elapsed / 1000.f;
            buzzer->base_info.input_info.freq = 1000.f + 2000.f * ratio;
            buzzer->base_info.input_info.volume = 80.f;
        }
        break;

    case BUZZER_START_SOUND:
        if (elapsed >= 1500U)
        {
            buzzer->sound_state = BUZZER_OFF;
            buzzer->base_info.input_info.freq = 0.f;
            buzzer->base_info.input_info.volume = 0.f;
        }
        else if (elapsed >= 1000U)
        {
            buzzer->base_info.input_info.freq = 8000.f;
            buzzer->base_info.input_info.volume = 50.f;
        }
        else if (elapsed >= 500U)
        {
            buzzer->base_info.input_info.freq = 2000.f;
            buzzer->base_info.input_info.volume = 50.f;
        }
        else
        {
            buzzer->base_info.input_info.freq = 1000.f;
            buzzer->base_info.input_info.volume = 50.f;
        }
        break;

    default:
        break;
    }

    float target_freq = buzzer->base_info.input_info.freq;
    float target_volume = buzzer->base_info.input_info.volume;

    if (target_freq <= 0 || target_volume <= buzzer->config.min_volume)
    {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0); // CCR=0，关闭输出
        return;
    }
    buzzer->base_info.tim_presc = Buzzer_Calc_Optimal_Presc(buzzer, target_freq);
    buzzer->base_info.ARR = Buzzer_Calc_ARR(buzzer);
    buzzer->base_info.duty = Buzzer_Volume_to_Duty(buzzer, target_volume);

    buzzer->base_info.CCR = Buzzer_Calc_CCR(buzzer);

    __HAL_TIM_PRESCALER(buzzer->config.tim, buzzer->base_info.tim_presc);                     // 先设分频（频率相关）
    __HAL_TIM_SET_AUTORELOAD(buzzer->config.tim, buzzer->base_info.ARR);                      // 再设ARR（频率相关）
    __HAL_TIM_SET_COMPARE(buzzer->config.tim, buzzer->config.channel, buzzer->base_info.CCR); // 最后设CCR（音量相关）
}

void Buzzer_Normal_On(void)
{
    buzzer.sound_state = BUZZER_NORMAL_ON;
    buzzer.sound_start_tick = HAL_GetTick();
}

void Buzzer_Normal_Off(void)
{
    buzzer.sound_state = BUZZER_OFF;
    buzzer.sound_start_tick = 0;
}

void Buzzer_Alarm(void)
{
    buzzer.sound_state = BUZZER_ALARM;
    buzzer.sound_start_tick = HAL_GetTick();
}

void Buzzer_StartSound(void)
{
    buzzer.sound_state = BUZZER_START_SOUND;
    buzzer.sound_start_tick = HAL_GetTick();
}
