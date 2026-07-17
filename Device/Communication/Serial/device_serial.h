#ifndef DEVICE_SERIAL_H
#define DEVICE_SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    DEVICE_SERIAL_RESULT_OK = 0,
    DEVICE_SERIAL_RESULT_INVALID_ARGUMENT,
    DEVICE_SERIAL_RESULT_UNSUPPORTED,
    DEVICE_SERIAL_RESULT_BUSY,
    DEVICE_SERIAL_RESULT_TIMEOUT,
    DEVICE_SERIAL_RESULT_IO_ERROR,
} Device_Serial_Result;

typedef enum {
    DEVICE_SERIAL_RX_ERROR_NONE = 0U,
    DEVICE_SERIAL_RX_ERROR_SOFTWARE_OVERFLOW = (1U << 0),
    DEVICE_SERIAL_RX_ERROR_HARDWARE_OVERRUN = (1U << 1),
    DEVICE_SERIAL_RX_ERROR_FRAMING = (1U << 2),
    DEVICE_SERIAL_RX_ERROR_PARITY = (1U << 3),
    DEVICE_SERIAL_RX_ERROR_DMA = (1U << 4),
} Device_Serial_RxError;

typedef struct {
    size_t transport_index;
    uint32_t expected_baud_rate;
    size_t minimum_tx_capacity;
    size_t minimum_rx_capacity;
    bool require_buffered_tx;
    bool require_buffered_rx;
} Device_Serial_Config;

typedef struct {
    uint32_t software_overflow_count;
    uint32_t hardware_overrun_count;
    uint32_t framing_error_count;
    uint32_t parity_error_count;
    uint32_t dma_error_count;
    uint32_t sticky_errors;
    size_t buffered_bytes;
    size_t buffer_capacity;
} Device_Serial_RxStatus;

typedef struct {
    const void *backend;
    bool initialized;
} Device_Serial;

bool Device_Serial_Init(
    Device_Serial *serial, const Device_Serial_Config *config);
Device_Serial_Result Device_Serial_WriteBuffered(
    Device_Serial *serial, const uint8_t *data, size_t length);
Device_Serial_Result Device_Serial_ReadAvailable(
    Device_Serial *serial, uint8_t *data, size_t capacity,
    size_t *transferred);
bool Device_Serial_GetRxStatus(
    Device_Serial *serial, Device_Serial_RxStatus *status);
bool Device_Serial_ClearRxStatus(Device_Serial *serial);
bool Device_Serial_Recover(Device_Serial *serial);

#endif
