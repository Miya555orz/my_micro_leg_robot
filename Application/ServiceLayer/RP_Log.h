/**
  ******************************************************************************
  * File Name          : RP_Log.h
  * Description        : Ring Buffer Log System
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 SZU RobotPilots LYQ .
  ******************************************************************************
  *
  * ==============================================================================
                       ##### How To Use #####
  * ==============================================================================
  * (#) 可配置选项: RP_LogConfigParam_t 结构体
  *     重点: output_range, use_timestamp, rtt_use_color
  *
  * (#) 添加对应库头文件，例如：
  *    #include "usart.h"
  *    #include "stm32f4xx_hal.h"
  *
  * (#) 串口输出接口实现（用户需实现以下函数）
  *     在RP_Log.c中实现 RP_Log_Transmit 函数，参考示例如下：
  *
  *     HAL库DMA示例:
  *     int RP_Log_Transmit(const uint8_t *data, uint16_t length)
  *     {
  *         if (HAL_UART_Transmit_DMA(&huart1, data, length) == HAL_OK) {
  *             return 0;
  *         }
  *         return -1;
  *     }
  *
  * (#) 日志输出接口（在任意文件中包含此头文件即可使用）
  *     直接使用 RP_LOG_XXX 宏输出日志，示例如下：
  *     RP_LOG_INFO("System started");
  *     RP_LOG_WARN("Battery low");
  *
  * (#) 日志线程调用（在独立线程中循环调用）
  *     while(1) {
  *         g_rp_log.work(&g_rp_log);
  *         osDelay(1);
  *     }
  *
  * (#) RTT输出配置（在 RP_Log.c 中设置 RP_LOG_USE_RTT 为 1 启用）
  *     需要在项目中集成 SEGGER_RTT 库
  *
  * ==============================================================================
                       ##### Working Principle #####
  * ==============================================================================
  * - 调用 g_rp_log.write() 时，日志内容被写入环形缓冲区（不阻塞）
  * - 在独立日志线程中循环调用 g_rp_log.work() 从环形缓冲区取出日志并发送
  * - 这种设计避免了串口正忙导致的日志丢失问题
  * - 支持RTT输出（需要使能 RP_LOG_USE_RTT）
  *
  * ==============================================================================
                       ##### Output Format #####
  * ==============================================================================
  * [时间戳][等级][文件:行号]: 用户内容
  * 例: [1234][INFO][main.c:45]: System initialized
  *
  ******************************************************************************
  */

#ifndef __RP_LOG_H
#define __RP_LOG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdarg.h>

    /* Includes ------------------------------------------------------------------*/

    /* Exported types ------------------------------------------------------------*/

    // 日志等级定义
    typedef enum
    {
        RP_LOG_LEVEL_FATAL = 0, // 致命错误
        RP_LOG_LEVEL_ERROR,     // 错误
        RP_LOG_LEVEL_WARN,      // 警告
        RP_LOG_LEVEL_INFO,      // 信息
        RP_LOG_LEVEL_DEBUG,     // 调试
        RP_LOG_LEVEL_TRACE      // 跟踪
    } RP_LogLevel_t;

    // 日志输出范围
    typedef enum
    {
        RP_LOG_OUTPUT_FATAL_ONLY = 0, // 仅输出FATAL
        RP_LOG_OUTPUT_FATAL_TO_ERROR, // 输出FATAL到ERROR
        RP_LOG_OUTPUT_FATAL_TO_WARN,  // 输出FATAL到WARN
        RP_LOG_OUTPUT_FATAL_TO_INFO,  // 输出FATAL到INFO
        RP_LOG_OUTPUT_FATAL_TO_DEBUG, // 输出FATAL到DEBUG
        RP_LOG_OUTPUT_ALL             // 输出所有级别
    } RP_LogOutputRange_t;

/*Config param start----------------------------------------------------------*/
#define RP_LOG_ENTRY_MAX_SIZE 256 // 单条日志最大长度
#define RP_LOG_RING_BUFFER_CNT 16 // 环形缓冲区可存储的日志条目数量

    // 配置参数结构体
    typedef struct
    {
        RP_LogOutputRange_t output_range; // 日志输出范围
        uint8_t use_timestamp;            // 是否使用时间戳（1=启用，0=禁用）
        uint8_t rtt_use_color;            // RTT是否使用颜色（1=启用，0=禁用）
    } RP_LogConfigParam_t;

    /*Config param end------------------------------------------------------------*/

    // 环形缓冲区条目
    typedef struct
    {
        uint8_t data[RP_LOG_ENTRY_MAX_SIZE]; // 日志数据
        uint16_t length;                     // 数据长度
    } RP_LogEntry_t;

    // 环形缓冲区结构体
    typedef struct
    {
        RP_LogEntry_t entries[RP_LOG_RING_BUFFER_CNT]; // 日志条目数组
        volatile uint16_t head;                        // 写指针
        volatile uint16_t tail;                        // 读指针
        volatile uint16_t count;                       // 当前日志数量
    } RP_LogRingBuffer_t;

    // 日志模块主结构体（函数指针API）
    typedef struct RP_Log_struct_t
    {
        RP_LogConfigParam_t config_param; // 可配置参数
        RP_LogRingBuffer_t ring_buffer;   // 环形缓冲区

        int (*write)(struct RP_Log_struct_t *log, RP_LogLevel_t level, const char *file, int line, const char *format, ...); // 写日志
        void (*work)(struct RP_Log_struct_t *log);                                                                           // 处理输出
        uint16_t (*get_count)(struct RP_Log_struct_t *log);                                                                  // 获取数量
        void (*flush)(struct RP_Log_struct_t *log);                                                                          // 清空缓冲区
    } RP_Log_t;

    /* Exported variables --------------------------------------------------------*/
    extern RP_Log_t g_rp_log;

    /* Exported functions --------------------------------------------------------*/

    // 串口发送接口（用户需实现此函数）
    int RP_Log_Transmit(const uint8_t *data, uint16_t length);

    /* User macros --------------------------------------------------------------*/

#define RP_LOG_FATAL(format, ...) g_rp_log.write(&g_rp_log, RP_LOG_LEVEL_FATAL, __FILE__, __LINE__, format, ##__VA_ARGS__)
#define RP_LOG_ERROR(format, ...) g_rp_log.write(&g_rp_log, RP_LOG_LEVEL_ERROR, __FILE__, __LINE__, format, ##__VA_ARGS__)
#define RP_LOG_WARN(format, ...) g_rp_log.write(&g_rp_log, RP_LOG_LEVEL_WARN, __FILE__, __LINE__, format, ##__VA_ARGS__)
#define RP_LOG_INFO(format, ...) g_rp_log.write(&g_rp_log, RP_LOG_LEVEL_INFO, __FILE__, __LINE__, format, ##__VA_ARGS__)
#define RP_LOG_DEBUG(format, ...) g_rp_log.write(&g_rp_log, RP_LOG_LEVEL_DEBUG, __FILE__, __LINE__, format, ##__VA_ARGS__)
#define RP_LOG_TRACE(format, ...) g_rp_log.write(&g_rp_log, RP_LOG_LEVEL_TRACE, __FILE__, __LINE__, format, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* __RP_LOG_H */
