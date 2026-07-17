#include "device_serial.h"

#include "bsp_uart.h"

static Device_Serial_Result map_result(BSP_UART_Result result)
{
    switch (result) {
    case BSP_UART_RESULT_OK:
        return DEVICE_SERIAL_RESULT_OK;
    case BSP_UART_RESULT_INVALID_ARGUMENT:
        return DEVICE_SERIAL_RESULT_INVALID_ARGUMENT;
    case BSP_UART_RESULT_UNSUPPORTED:
        return DEVICE_SERIAL_RESULT_UNSUPPORTED;
    case BSP_UART_RESULT_BUSY:
        return DEVICE_SERIAL_RESULT_BUSY;
    case BSP_UART_RESULT_TIMEOUT:
        return DEVICE_SERIAL_RESULT_TIMEOUT;
    case BSP_UART_RESULT_IO_ERROR:
    default:
        return DEVICE_SERIAL_RESULT_IO_ERROR;
    }
}

static BSP_UART_Handle backend(Device_Serial *serial)
{
    return ((serial != NULL) && serial->initialized) ?
        (BSP_UART_Handle)serial->backend : NULL;
}

bool Device_Serial_Init(
    Device_Serial *serial, const Device_Serial_Config *config)
{
    BSP_UART_Info info;
    BSP_UART_Handle handle;

    if (serial != NULL) {
        *serial = (Device_Serial){0};
    }
    if ((serial == NULL) || (config == NULL)) {
        return false;
    }
    handle = BSP_UART_GetHandle(config->transport_index);
    if ((handle == NULL) || !BSP_UART_GetInfo(handle, &info) ||
        (info.baud_rate != config->expected_baud_rate) ||
        (config->require_buffered_tx && !info.supports_buffered_tx) ||
        (config->require_buffered_rx && !info.supports_buffered_rx) ||
        (info.tx_buffer_capacity < config->minimum_tx_capacity) ||
        (info.rx_buffer_capacity < config->minimum_rx_capacity)) {
        return false;
    }
    serial->backend = handle;
    serial->initialized = true;
    return true;
}

Device_Serial_Result Device_Serial_WriteBuffered(
    Device_Serial *serial, const uint8_t *data, size_t length)
{
    BSP_UART_Handle handle = backend(serial);

    return (handle == NULL) ? DEVICE_SERIAL_RESULT_INVALID_ARGUMENT :
        map_result(BSP_UART_WriteBuffered(handle, data, length));
}

Device_Serial_Result Device_Serial_ReadAvailable(
    Device_Serial *serial, uint8_t *data, size_t capacity,
    size_t *transferred)
{
    BSP_UART_Handle handle = backend(serial);

    return (handle == NULL) ? DEVICE_SERIAL_RESULT_INVALID_ARGUMENT :
        map_result(BSP_UART_ReadAvailable(
            handle, data, capacity, transferred));
}

bool Device_Serial_GetRxStatus(
    Device_Serial *serial, Device_Serial_RxStatus *status)
{
    BSP_UART_Handle handle = backend(serial);
    BSP_UART_RxStatus source;

    if ((handle == NULL) || (status == NULL) ||
        !BSP_UART_GetRxStatus(handle, &source)) {
        return false;
    }
    *status = (Device_Serial_RxStatus){
        .software_overflow_count = source.software_overflow_count,
        .hardware_overrun_count = source.hardware_overrun_count,
        .framing_error_count = source.framing_error_count,
        .parity_error_count = source.parity_error_count,
        .dma_error_count = source.dma_error_count,
        .sticky_errors = source.sticky_errors,
        .buffered_bytes = source.buffered_bytes,
        .buffer_capacity = source.buffer_capacity,
    };
    return true;
}

bool Device_Serial_ClearRxStatus(Device_Serial *serial)
{
    BSP_UART_Handle handle = backend(serial);

    return (handle != NULL) && BSP_UART_ClearRxStatus(handle);
}

bool Device_Serial_Recover(Device_Serial *serial)
{
    BSP_UART_Handle handle = backend(serial);

    return (handle != NULL) && BSP_UART_Recover(handle);
}
