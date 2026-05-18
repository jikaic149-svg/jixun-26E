#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

void BSP_UART_Init(void) {
    // SysConfig 将会自动生成 SYSCFG_DL_UART_0_init() 并在系统初始化时调用
    // 同时也会把 DMA 初始化好。
    // 我们只需要在这里开启串口中断即可（用于空闲中断等接收处理）
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void BSP_UART_SendChar(uint8_t ch) {
    DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
}

void BSP_UART_SendString(char *str) {
    while (*str) {
        BSP_UART_SendChar((uint8_t)*str++);
    }
}

// === DMA 发送函数 ===
// 前提：在 SysConfig 中勾选 UART0 的 "Enable TX DMA Channel"
void BSP_UART_SendDMA(uint8_t *data, uint16_t len) {
    DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
    // 设置 DMA 源地址 (数据内存数组)
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)data);
    // 设置 DMA 目的地址 (UART 的 TX 寄存器，SysConfig已默认设好，通常不需要每次改)
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_0_INST->TXDATA));
    // 设置要发送的数据长度
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, len);
    
    // 启用通道，DMA 瞬间接管传输
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
}

void BSP_UART1_Init(void) {
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

void BSP_UART1_SendChar(uint8_t ch) {
    DL_UART_Main_transmitDataBlocking(UART_1_INST, ch);
}

void BSP_UART1_SendString(char *str) {
    while (*str) {
        BSP_UART1_SendChar((uint8_t)*str++);
    }
}

void BSP_UART1_SendDMA(uint8_t *data, uint16_t len) {
    DL_DMA_disableChannel(DMA, DMA_CH3_CHAN_ID);
    DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_TX);
    DL_DMA_setSrcAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)data);
    DL_DMA_setDestAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)(&UART_1_INST->TXDATA));
    DL_DMA_setTransferSize(DMA, DMA_CH3_CHAN_ID, len);
    DL_DMA_enableChannel(DMA, DMA_CH3_CHAN_ID);
}

void BSP_UART2_Init(void) {
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void BSP_UART2_SendChar(uint8_t ch) {
    DL_UART_Main_transmitDataBlocking(UART_2_INST, ch);
}

void BSP_UART2_SendString(char *str) {
    while (*str) {
        BSP_UART2_SendChar((uint8_t)*str++);
    }
}

void BSP_UART2_SendDMA(uint8_t *data, uint16_t len) {
    DL_DMA_disableChannel(DMA, DMA_CH5_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH5_CHAN_ID, (uint32_t)data);
    DL_DMA_setDestAddr(DMA, DMA_CH5_CHAN_ID, (uint32_t)(&UART_2_INST->TXDATA));
    DL_DMA_setTransferSize(DMA, DMA_CH5_CHAN_ID, len);
    DL_DMA_enableChannel(DMA, DMA_CH5_CHAN_ID);
}
