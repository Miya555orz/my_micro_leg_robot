/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2021 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************

File    : SEGGER_RTT_Conf.h
Purpose : Implementation of SEGGER real-time transfer (RTT) which
          allows real-time communication on targets which support
          debugger memory accesses while the CPU is running.
Revision: $Rev: 21386 $

*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 *
 *       Defines, configurable
 *
 **********************************************************************
 */

// 缓冲区数量配置
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS (3)   // 上行通道数（目标到主机）
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS (3) // 下行通道数（主机到目标）

// 缓冲区大小配置
#define BUFFER_SIZE_UP (2048) // 上行缓冲区大小
#define BUFFER_SIZE_DOWN (16) // 下行缓冲区大小

// 模式配置
#define SEGGER_RTT_MODE_DEFAULT SEGGER_RTT_MODE_NO_BLOCK_SKIP

// 内存分配
#ifndef SEGGER_RTT_MEMCPY_USE_BYTELOOP
#define SEGGER_RTT_MEMCPY_USE_BYTELOOP 0 // 使用优化的 memcpy
#endif

// RTT 控制块放置（用于 OpenOCD 搜索）
#define SEGGER_RTT_ALIGNMENT 0 // 不强制对齐

#ifdef __cplusplus
}
#endif

#endif

/*************************** End of file ****************************/
