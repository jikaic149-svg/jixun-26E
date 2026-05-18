#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

void BSP_UART_Init(void);
void BSP_UART_SendChar(uint8_t ch);
void BSP_UART_SendString(char *str);
void BSP_UART_SendDMA(uint8_t *data, uint16_t len);

void BSP_UART1_Init(void);
void BSP_UART1_SendChar(uint8_t ch);
void BSP_UART1_SendString(char *str);
void BSP_UART1_SendDMA(uint8_t *data, uint16_t len);

void BSP_UART2_Init(void);
void BSP_UART2_SendChar(uint8_t ch);
void BSP_UART2_SendString(char *str);
void BSP_UART2_SendDMA(uint8_t *data, uint16_t len);

#endif // BSP_UART_H
