/*
 * isr.c: 中断服务例程集中管理 (Rule 5: 确定性 RTOS 调度与中断收束)
 * 所有的硬件中断服务例程强制集中在此文件中。
 * 严禁在 ISR 中加入耗时业务逻辑，仅允许做标记或轻量级数据转存。
 */

#include "ti_msp_dl_config.h"
#include "../Device/device_init.h" // 引入 bt_inst 实例

void UART0_IRQHandler(void)
{
    // 获取当前最高优先级的中断标志
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            // 读取接收数据寄存器 (硬件会自动清除 RX 中断标志)
            {
                uint8_t rx_data = DL_UART_Main_receiveData(UART_0_INST);
                // 将数据喂给 Device 层的协议解析器
                Bluetooth_FeedRxByte(&bt_inst, rx_data);
            }
            break;
            
        default:
            break;
    }
}

void UART1_IRQHandler(void)
{
    // 获取当前最高优先级的中断标志
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            // 读取接收数据寄存器 (硬件会自动清除 RX 中断标志)
            {
                uint8_t rx_data = DL_UART_Main_receiveData(UART_1_INST);
                // 暂时将数据喂给同一个协议解析器 (如有独立的逻辑请在此修改)
                Bluetooth_FeedRxByte(&bt_inst, rx_data);
            }
            break;
            
        default:
            break;
    }
}

void UART4_IRQHandler(void)
{
    // 获取当前最高优先级的中断标志
    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            {
                uint8_t rx_data = DL_UART_Main_receiveData(UART_2_INST);
                JY61P_FIFO_Write(&jy61p_inst, rx_data);
            }
            break;
            
        default:
            break;
    }
}
