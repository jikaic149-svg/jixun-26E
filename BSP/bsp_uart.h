#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef const struct BSP_UART_Resource *BSP_UART_Handle;

typedef enum {
    BSP_UART_RESULT_OK = 0,
    BSP_UART_RESULT_INVALID_ARGUMENT,
    BSP_UART_RESULT_UNSUPPORTED,
    BSP_UART_RESULT_BUSY,
    BSP_UART_RESULT_TIMEOUT,
    BSP_UART_RESULT_IO_ERROR,
} BSP_UART_Result;

typedef enum {
    BSP_UART_RX_ERROR_NONE = 0U,
    BSP_UART_RX_ERROR_SOFTWARE_OVERFLOW = (1U << 0),
    BSP_UART_RX_ERROR_HARDWARE_OVERRUN = (1U << 1),
    BSP_UART_RX_ERROR_FRAMING = (1U << 2),
    BSP_UART_RX_ERROR_PARITY = (1U << 3),
    BSP_UART_RX_ERROR_DMA = (1U << 4),
} BSP_UART_RxError;

typedef struct {
    uint32_t software_overflow_count;
    uint32_t hardware_overrun_count;
    uint32_t framing_error_count;
    uint32_t parity_error_count;
    uint32_t dma_error_count;
    uint32_t sticky_errors;
    size_t buffered_bytes;
    size_t buffer_capacity;
} BSP_UART_RxStatus;

typedef struct {
    uint32_t baud_rate;
    bool supports_tx_dma;
    bool supports_rx_dma;
    bool supports_buffered_tx;
    bool supports_buffered_rx;
    size_t tx_buffer_capacity;
    size_t rx_buffer_capacity;
} BSP_UART_Info;

typedef void (*BSP_UART_TransferCallback)(
    BSP_UART_Handle uart,
    BSP_UART_Result result,
    size_t transferred,
    void *context);

/* Return the number of logical UART transports provided by this BSP. */
size_t BSP_UART_Count(void);

/* Return a program-lifetime logical handle, or NULL for an invalid index. */
BSP_UART_Handle BSP_UART_GetHandle(size_t index);

/* Read immutable configuration/capabilities. Failure leaves info unchanged. */
bool BSP_UART_GetInfo(BSP_UART_Handle uart, BSP_UART_Info *info);

/* Transfer exactly length bytes unless a timeout/error occurs. The result is
 * OK only for a complete transfer. transferred is required, is set to zero
 * before a recognized transfer begins, and reports bytes consumed from the
 * caller buffer including on timeout. timeout_ms is one deadline for the whole
 * call, not a fresh timeout per byte; zero performs no waiting. TX and RX may
 * run concurrently, but a blocking call returns BUSY while DMA owns the same
 * direction. data may be NULL only when length is zero. */
BSP_UART_Result BSP_UART_WriteBlocking(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    size_t *transferred);

/* Atomically enqueue one complete frame for interrupt-driven transmission.
 * This call never waits. BUSY means the complete frame did not fit and no
 * byte from it was accepted, so callers can safely drop or retry the frame
 * without corrupting the byte stream. */
BSP_UART_Result BSP_UART_WriteBuffered(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length);

BSP_UART_Result BSP_UART_ReadBlocking(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    size_t *transferred);

/* Drain up to capacity bytes already buffered by the BSP RX ISR/DMA backend.
 * This call never waits. transferred is required and distinguishes an empty
 * valid buffer (OK, zero) from an invalid/unsupported transport. It returns
 * BUSY while an explicit RX DMA transfer owns the receive direction. */
BSP_UART_Result BSP_UART_ReadAvailable(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t capacity,
    size_t *transferred);

/* Atomically snapshot sticky RX error flags/saturating counters and occupancy.
 * ClearRxStatus atomically clears only the sticky flags/counters; it does not
 * discard bytes already buffered for the caller. A DMA backend may also
 * re-arm its RX channel and discard an uncommitted damaged staging fragment.
 * Failure leaves status unchanged. */
bool BSP_UART_GetRxStatus(
    BSP_UART_Handle uart, BSP_UART_RxStatus *status);
bool BSP_UART_ClearRxStatus(BSP_UART_Handle uart);

/* Abort buffered transfers, flush UART FIFOs, reset the software rings and
 * restart interrupts/DMA without changing the configured baud rate or pins. */
bool BSP_UART_Recover(BSP_UART_Handle uart);

/* Start an optional zero-copy DMA transfer. The caller owns the buffer and
 * must keep it alive and unchanged (TX) or writable (RX) until exactly one
 * completion callback. timeout_ms is a total transfer deadline and must be
 * nonzero; the BSP must stop/reset a stuck backend and release ownership on
 * timeout.
 *
 * Only an OK Start result accepts the buffer; any immediate failure produces
 * no callback. An accepted transfer produces exactly one callback, never
 * inline before Start returns and always in ISR context. The BSP clears busy
 * before invoking it. Callback result OK means transferred == length; timeout,
 * cancellation, or I/O failure may report a partial count. The callback only
 * records/notifies completion and must not parse data, call blocking APIs, or
 * start another transfer. */
BSP_UART_Result BSP_UART_StartWriteDMA(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    BSP_UART_TransferCallback callback,
    void *context);

BSP_UART_Result BSP_UART_StartReadDMA(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    BSP_UART_TransferCallback callback,
    void *context);

/* Invalid handles read as not busy; validate handles through GetInfo first. */
bool BSP_UART_IsTxBusy(BSP_UART_Handle uart);
bool BSP_UART_IsRxBusy(BSP_UART_Handle uart);

#endif
