with open("../../../Application/ServiceLayer/RP_Log.c", "r", encoding="utf-8") as f:
    text = f.read()

import re
text = re.sub(r'__attribute__\(\(weak\)\) int RP_Log_Transmit\(const uint8_t \*data, uint16_t length\).*', '''__attribute__((weak)) int RP_Log_Transmit(const uint8_t *data, uint16_t length)
{
    // 如果串口正忙，返回-1让上层把数据放回环形缓冲区
    if (huart9.gState != HAL_UART_STATE_READY)
    {
        return -1;
    }

    // 清除 D-Cache 数据，确保 DMA 能读到最新的内容，防止缓存一致性问题
    SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)data) & ~(uint32_t)0x1F), length + 32);

    if (HAL_UART_Transmit_DMA(&huart9, data, length) == HAL_OK)
    {
        return 0;
    }

    return -1;
}''', text, flags=re.DOTALL)

with open("../../../Application/ServiceLayer/RP_Log.c", "w", encoding="utf-8") as f:
    f.write(text)
