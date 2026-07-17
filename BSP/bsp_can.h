#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_CAN_CLASSIC_MAX_DATA_LENGTH 8U
#define BSP_CAN_FD_MAX_DATA_LENGTH      64U

typedef const struct BSP_CAN_Resource *BSP_CAN_Handle;

typedef enum {
    BSP_CAN_RESULT_OK = 0,
    BSP_CAN_RESULT_NO_DATA,
    BSP_CAN_RESULT_BUSY,
    BSP_CAN_RESULT_TIMEOUT,
    BSP_CAN_RESULT_BUS_OFF,
    BSP_CAN_RESULT_INVALID_ARGUMENT,
    BSP_CAN_RESULT_UNSUPPORTED,
    BSP_CAN_RESULT_IO_ERROR,
} BSP_CAN_Result;

typedef struct {
    uint32_t identifier;
    uint8_t data[BSP_CAN_FD_MAX_DATA_LENGTH];
    size_t data_length;
    bool extended_identifier;
    bool fd_format;
    bool bit_rate_switch;
} BSP_CAN_Frame;

typedef struct {
    uint32_t nominal_bit_rate;
    uint32_t data_bit_rate;
    bool supports_fd;
    bool supports_bit_rate_switch;
} BSP_CAN_Info;

/* A handle identifies one logical CAN bus. Instance, RX/TX pins, message RAM,
 * filters and interrupt routing belong to the MCU BSP/SysConfig only. */
size_t BSP_CAN_Count(void);
BSP_CAN_Handle BSP_CAN_GetHandle(size_t index);
bool BSP_CAN_GetInfo(BSP_CAN_Handle can, BSP_CAN_Info *info);

BSP_CAN_Result BSP_CAN_Send(
    BSP_CAN_Handle can, const BSP_CAN_Frame *frame, uint32_t timeout_ms);
BSP_CAN_Result BSP_CAN_ReadAvailable(
    BSP_CAN_Handle can, BSP_CAN_Frame *frame);
bool BSP_CAN_IsBusOff(BSP_CAN_Handle can);
BSP_CAN_Result BSP_CAN_Recover(BSP_CAN_Handle can);

#endif
