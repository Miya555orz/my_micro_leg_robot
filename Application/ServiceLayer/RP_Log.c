/**
 ******************************************************************************
 * File Name          : RP_Log.c
 * Description        : Ring Buffer Log System
 ******************************************************************************
 * @attention
 * Copyright (c) 2026 SZU RobotPilots.
 *
 * 使用环形缓冲区实现无阻塞日志写入，避免串口正忙导致的日志丢失
 * 支持RTT输出（需设置 RP_LOG_USE_RTT 为 1）
 * 串口发送需用户实现 RP_Log_Transmit 函数
 *
 ******************************************************************************
 */

#include "RP_Log.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/

#define RP_LOG_USE_RTT 1 // RTT输出配置（启用请将此值改为 1）

#if RP_LOG_USE_RTT
#include "SEGGER_RTT.h"
#endif

/* Private variables --------------------------------------------------------*/

static const char *g_level_names[] = { // 日志等级字符串
    "FATAL", "ERROR", "WARN ", "INFO ", "DEBUG", "TRACE"};

#if RP_LOG_USE_RTT
#define RP_LOG_COLOR_FATAL "\033[1;35m"
#define RP_LOG_COLOR_ERROR "\033[1;31m"
#define RP_LOG_COLOR_WARN "\033[1;33m"
#define RP_LOG_COLOR_INFO "\033[1;32m"
#define RP_LOG_COLOR_DEBUG "\033[1;36m"
#define RP_LOG_COLOR_TRACE "\033[0;37m"
#define RP_LOG_COLOR_RESET "\033[0m"

static const char *g_level_colors[] = { // 日志颜色
    RP_LOG_COLOR_FATAL, RP_LOG_COLOR_ERROR, RP_LOG_COLOR_WARN,
    RP_LOG_COLOR_INFO, RP_LOG_COLOR_DEBUG, RP_LOG_COLOR_TRACE};
#endif

/* Private function prototypes ----------------------------------------------*/

static int RP_Log_Write(RP_Log_t *log, RP_LogLevel_t level, const char *file, int line, const char *format, ...); // 写日志
static void RP_Log_Work(RP_Log_t *log);                                                                           // 处理输出
static uint16_t RP_Log_GetCount(RP_Log_t *log);                                                                   // 获取数量
static void RP_Log_Flush(RP_Log_t *log);                                                                          // 清空缓冲区

static uint16_t RB_GetNextIndex(uint16_t index, uint16_t max);                    // 计算下一个索引
static int RB_IsFull(RP_LogRingBuffer_t *rb);                                     // 判断缓冲区满
static int RB_IsEmpty(RP_LogRingBuffer_t *rb);                                    // 判断缓冲区空
static int RB_Push(RP_LogRingBuffer_t *rb, const uint8_t *data, uint16_t length); // 写入数据
static int RB_Pop(RP_LogRingBuffer_t *rb, uint8_t *data, uint16_t *length);       // 读取数据

#if RP_LOG_USE_RTT
static void RP_Log_RttOutput(RP_LogLevel_t level, const char *file, int line, const char *format, va_list args); // RTT输出
#endif
/* Public variables --------------------------------------------------------*/

// 日志全局实例（函数指针初始化）
RP_Log_t g_rp_log = {
    .config_param = {
        .output_range = RP_LOG_OUTPUT_ALL,
        .use_timestamp = 1,
        .rtt_use_color = 1},
    .ring_buffer = {0},

    .write = RP_Log_Write,
    .work = RP_Log_Work,
    .get_count = RP_Log_GetCount,
    .flush = RP_Log_Flush,
};
/* Private functions --------------------------------------------------------*/

// 计算下一个索引
static uint16_t RB_GetNextIndex(uint16_t index, uint16_t max)
{
    return (index + 1) % max;
}

// 判断缓冲区满
static int RB_IsFull(RP_LogRingBuffer_t *rb)
{
    return rb->count >= RP_LOG_RING_BUFFER_CNT;
}

// 判断缓冲区空
static int RB_IsEmpty(RP_LogRingBuffer_t *rb)
{
    return rb->count == 0;
}

// 写入数据
static int RB_Push(RP_LogRingBuffer_t *rb, const uint8_t *data, uint16_t length)
{
    if (RB_IsFull(rb))
    {
        return -1;
    }

    if (length > RP_LOG_ENTRY_MAX_SIZE)
    {
        length = RP_LOG_ENTRY_MAX_SIZE;
    }

    memcpy(rb->entries[rb->head].data, data, length);
    rb->entries[rb->head].length = length;
    rb->head = RB_GetNextIndex(rb->head, RP_LOG_RING_BUFFER_CNT);
    rb->count++;

    return 0;
}

// 读取数据
static int RB_Pop(RP_LogRingBuffer_t *rb, uint8_t *data, uint16_t *length)
{
    if (RB_IsEmpty(rb))
    {
        return -1;
    }

    *length = rb->entries[rb->tail].length;
    memcpy(data, rb->entries[rb->tail].data, *length);
    rb->tail = RB_GetNextIndex(rb->tail, RP_LOG_RING_BUFFER_CNT);
    rb->count--;

    return 0;
}

#if RP_LOG_USE_RTT
// RTT输出
static void RP_Log_RttOutput(RP_LogLevel_t level, const char *file, int line, const char *format, va_list args)
{
    static uint8_t rtt_buf[RP_LOG_ENTRY_MAX_SIZE];
    int rtt_len = 0;

    // 提取文件名
    const char *filename = file;
    const char *slash = strrchr(file, '\\');
    if (slash != NULL)
    {
        filename = slash + 1;
    }
    else
    {
        slash = strrchr(file, '/');
        if (slash != NULL)
        {
            filename = slash + 1;
        }
    }

    // 颜色前缀
    if (g_rp_log.config_param.rtt_use_color)
    {
        rtt_len += snprintf((char *)rtt_buf, RP_LOG_ENTRY_MAX_SIZE,
                            "%s", g_level_colors[level]);
    }

    // 时间戳
    if (g_rp_log.config_param.use_timestamp)
    {
#if defined(USE_HAL_DRIVER)
        rtt_len += snprintf((char *)rtt_buf + rtt_len, RP_LOG_ENTRY_MAX_SIZE - rtt_len,
                            "[%u] ", HAL_GetTick());
#endif
    }

    // 等级和位置
    rtt_len += snprintf((char *)rtt_buf + rtt_len, RP_LOG_ENTRY_MAX_SIZE - rtt_len,
                        "[%s][%s:%d]: ", g_level_names[level], filename, line);

    // 颜色重置
    if (g_rp_log.config_param.rtt_use_color)
    {
        rtt_len += snprintf((char *)rtt_buf + rtt_len, RP_LOG_ENTRY_MAX_SIZE - rtt_len,
                            "%s", RP_LOG_COLOR_RESET);
    }

    // 用户内容
    rtt_len += vsnprintf((char *)rtt_buf + rtt_len, RP_LOG_ENTRY_MAX_SIZE - rtt_len, format, args);

    // 溢出保护
    if (rtt_len >= RP_LOG_ENTRY_MAX_SIZE - 2)
    {
        rtt_len = RP_LOG_ENTRY_MAX_SIZE - 3;
    }

    rtt_buf[rtt_len++] = '\r';
    rtt_buf[rtt_len++] = '\n';

    SEGGER_RTT_Write(0, rtt_buf, rtt_len);
}
#endif

/* Public functions --------------------------------------------------------*/

/**
 * @brief  写入日志到环形缓冲区
 * @param  log: 日志模块实例指针
 * @param  level: 日志等级
 * @param  file: 源文件名
 * @param  line: 行号
 * @param  format: 格式化字符串
 * @retval 0=成功, -1=失败
 */
static int RP_Log_Write(RP_Log_t *log, RP_LogLevel_t level, const char *file, int line, const char *format, ...)
{
    if (log == NULL)
    {
        return -1;
    }

    // 等级过滤
    switch (log->config_param.output_range)
    {
    case RP_LOG_OUTPUT_FATAL_ONLY:
        if (level != RP_LOG_LEVEL_FATAL)
            return -1;
        break;
    case RP_LOG_OUTPUT_FATAL_TO_ERROR:
        if (level > RP_LOG_LEVEL_ERROR)
            return -1;
        break;
    case RP_LOG_OUTPUT_FATAL_TO_WARN:
        if (level > RP_LOG_LEVEL_WARN)
            return -1;
        break;
    case RP_LOG_OUTPUT_FATAL_TO_INFO:
        if (level > RP_LOG_LEVEL_INFO)
            return -1;
        break;
    case RP_LOG_OUTPUT_FATAL_TO_DEBUG:
        if (level > RP_LOG_LEVEL_DEBUG)
            return -1;
        break;
    case RP_LOG_OUTPUT_ALL:
    default:
        break;
    }

    // 提取文件名
    const char *filename = file;
    const char *slash = strrchr(file, '\\');
    if (slash != NULL)
    {
        filename = slash + 1;
    }
    else
    {
        slash = strrchr(file, '/');
        if (slash != NULL)
        {
            filename = slash + 1;
        }
    }

    // 格式化日志内容
    uint8_t buffer[RP_LOG_ENTRY_MAX_SIZE];
    int len = 0;

    // 时间戳
    if (log->config_param.use_timestamp)
    {
#if defined(USE_HAL_DRIVER)
        len += snprintf((char *)buffer + len, RP_LOG_ENTRY_MAX_SIZE - len,
                        "[%u] ", HAL_GetTick());
#endif
    }

    // 等级和位置
    len += snprintf((char *)buffer + len, RP_LOG_ENTRY_MAX_SIZE - len,
                    "[%s][%s:%d]: ", g_level_names[level], filename, line);

    // 用户内容
    va_list args;
    va_start(args, format);
    len += vsnprintf((char *)buffer + len, RP_LOG_ENTRY_MAX_SIZE - len, format, args);
    va_end(args);

    // 溢出保护
    if (len >= RP_LOG_ENTRY_MAX_SIZE - 2)
    {
        len = RP_LOG_ENTRY_MAX_SIZE - 3;
    }

    // 换行
    buffer[len++] = '\r';
    buffer[len++] = '\n';
#if RP_LOG_USE_RTT
    // 先RTT输出再写入环形缓冲区，防止串口卡死导致RTT也卡死
    // 可以试试把下面三行代码放在RB_Push后面，看看卡死的效果
    va_start(args, format);
    RP_Log_RttOutput(level, file, line, format, args);
    va_end(args);
#endif
    // 写入环形缓冲区
    if (RB_Push(&log->ring_buffer, buffer, (uint16_t)len) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  处理日志输出（需在独立线程中循环调用）
 * @param  log: 日志模块实例指针
 * @retval None
 */
static void RP_Log_Work(RP_Log_t *log)
{
    if (log == NULL)
    {
        return;
    }

    // 取出日志并发送
    __attribute__((section(".AXI_SRAM"))) static uint8_t buffer[RP_LOG_ENTRY_MAX_SIZE];
    uint16_t length;

    if (RB_Pop(&log->ring_buffer, buffer, &length) == 0)
    {
        if (RP_Log_Transmit(buffer, length) != 0)
        {
            // 发送失败，重新放回缓冲区
            RB_Push(&log->ring_buffer, buffer, length);
        }
    }
}

/**
 * @brief  获取环形缓冲区中可用的日志数量
 * @param  log: 日志模块实例指针
 * @retval 可用日志数量
 */
static uint16_t RP_Log_GetCount(RP_Log_t *log)
{
    if (log == NULL)
    {
        return 0;
    }
    return log->ring_buffer.count;
}

/**
 * @brief  清空环形缓冲区
 * @param  log: 日志模块实例指针
 * @retval None
 */
static void RP_Log_Flush(RP_Log_t *log)
{
    if (log == NULL)
    {
        return;
    }
    memset(&log->ring_buffer, 0, sizeof(RP_LogRingBuffer_t));
}

/* Weak functions ----------------------------------------------------------*/

/**
 * @brief  串口发送接口（用户需实现此函数）
 * @param  data: 待发送数据指针
 * @param  length: 数据长度
 * @retval 0=成功, -1=失败
 */

__attribute__((weak)) int RP_Log_Transmit(const uint8_t *data, uint16_t length)
{
    // 改用串口8进行日志发送，规避无硬件映射的UART9
    if (huart8.gState != HAL_UART_STATE_READY)
    {
        return -1;
    }

    if (HAL_UART_Transmit_DMA(&huart8, data, length) == HAL_OK)
    {
        return 0;
    }

    return -1;
}
