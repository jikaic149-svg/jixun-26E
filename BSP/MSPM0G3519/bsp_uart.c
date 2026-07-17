#include "bsp_uart.h"

#include <limits.h>
#include <stddef.h>

#include "bsp_resource_map.h"

#define BSP_UART_COUNT          3U
#define BSP_UART_RX_BUFFER_SIZE 128U
#define BSP_UART_RX_CAPACITY    (BSP_UART_RX_BUFFER_SIZE - 1U)
#define BSP_UART_TX_BUFFER_SIZE 1024U
#define BSP_UART_TX_CAPACITY    (BSP_UART_TX_BUFFER_SIZE - 1U)
#define BSP_UART_WAIT_STEPS_MS  100U
#define BSP_UART_DMA_RX_BUFFER_SIZE 64U

struct BSP_UART_Resource {
    UART_Regs *instance;
    IRQn_Type irq;
    uint32_t baud_rate;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint8_t rx_buffer[BSP_UART_RX_BUFFER_SIZE];
    volatile uint16_t tx_head;
    volatile uint16_t tx_tail;
    uint8_t tx_buffer[BSP_UART_TX_BUFFER_SIZE];
    uint8_t dma_rx_buffer[BSP_UART_DMA_RX_BUFFER_SIZE];
    volatile uint16_t dma_tx_length;
    uint8_t dma_rx_channel;
    uint8_t dma_tx_channel;
    bool uses_rx_dma;
    bool uses_tx_dma;
    volatile uint32_t software_overflow_count;
    volatile uint32_t hardware_overrun_count;
    volatile uint32_t framing_error_count;
    volatile uint32_t parity_error_count;
    volatile uint32_t dma_error_count;
    volatile uint32_t sticky_errors;
};

static struct BSP_UART_Resource g_uart[BSP_UART_COUNT] = {
    {.instance = BSP_RESOURCE_DEBUG_UART,
     .irq = BSP_RESOURCE_DEBUG_UART_IRQ,
     .baud_rate = BSP_RESOURCE_DEBUG_UART_BAUD},
    {.instance = BSP_RESOURCE_AUX_UART,
     .irq = BSP_RESOURCE_AUX_UART_IRQ,
     .baud_rate = BSP_RESOURCE_AUX_UART_BAUD},
    {.instance = BSP_RESOURCE_IMU_UART,
     .irq = BSP_RESOURCE_IMU_UART_IRQ,
     .baud_rate = BSP_RESOURCE_IMU_UART_BAUD,
     .dma_rx_channel = BSP_RESOURCE_IMU_DMA_RX,
     .uses_rx_dma = true},
};

static struct BSP_UART_Resource *uart_from_handle(BSP_UART_Handle uart)
{
    for (size_t index = 0U; index < BSP_UART_COUNT; ++index) {
        if (uart == &g_uart[index]) {
            return &g_uart[index];
        }
    }
    return NULL;
}

static void increment_saturating(volatile uint32_t *value)
{
    if (*value != UINT32_MAX) {
        ++(*value);
    }
}

static void uart_store_rx(struct BSP_UART_Resource *uart, uint8_t byte)
{
    uint16_t next = (uint16_t)((uart->rx_head + 1U) %
        BSP_UART_RX_BUFFER_SIZE);

    if (next == uart->rx_tail) {
        increment_saturating(&uart->software_overflow_count);
        uart->sticky_errors |= BSP_UART_RX_ERROR_SOFTWARE_OVERFLOW;
        return;
    }
    uart->rx_buffer[uart->rx_head] = byte;
    uart->rx_head = next;
}

static void uart_drain_fifo(struct BSP_UART_Resource *uart)
{
    while (!DL_UART_Main_isRXFIFOEmpty(uart->instance)) {
        uart_store_rx(uart,
            (uint8_t)DL_UART_Main_receiveData(uart->instance));
    }
}

static uint32_t dma_channel_interrupt(uint8_t channel)
{
    return UINT32_C(1) << channel;
}

static void uart_dma_start_rx(struct BSP_UART_Resource *uart)
{
    const uint32_t interrupt = dma_channel_interrupt(uart->dma_rx_channel);

    DL_DMA_disableChannel(DMA, uart->dma_rx_channel);
    DL_DMA_clearInterruptStatus(DMA, interrupt);
    DL_DMA_setSrcAddr(
        DMA, uart->dma_rx_channel, (uint32_t)&uart->instance->RXDATA);
    DL_DMA_setDestAddr(
        DMA, uart->dma_rx_channel, (uint32_t)uart->dma_rx_buffer);
    DL_DMA_setTransferSize(
        DMA, uart->dma_rx_channel, BSP_UART_DMA_RX_BUFFER_SIZE);
    DL_DMA_enableChannel(DMA, uart->dma_rx_channel);
}

static void uart_dma_commit_rx(struct BSP_UART_Resource *uart)
{
    uint16_t remaining;
    uint16_t received;

    DL_DMA_disableChannel(DMA, uart->dma_rx_channel);
    remaining = DL_DMA_getTransferSize(DMA, uart->dma_rx_channel);
    if (remaining > BSP_UART_DMA_RX_BUFFER_SIZE) {
        increment_saturating(&uart->dma_error_count);
        uart->sticky_errors |= BSP_UART_RX_ERROR_DMA;
        received = 0U;
    }
    else {
        received = (uint16_t)(BSP_UART_DMA_RX_BUFFER_SIZE - remaining);
    }
    for (uint16_t index = 0U; index < received; ++index) {
        uart_store_rx(uart, uart->dma_rx_buffer[index]);
    }
    uart_dma_start_rx(uart);
}

static void uart_dma_flush_partial_rx(struct BSP_UART_Resource *uart)
{
    const uint16_t remaining =
        DL_DMA_getTransferSize(DMA, uart->dma_rx_channel);

    /* Full buffers are normally submitted by DMA_IRQHandler and short
     * buffers by UART RX-timeout.  Sampling the remaining count here is a
     * bounded fallback for UART implementations whose timeout flag is
     * suppressed after DMA drains the RX FIFO. */
    if (remaining < BSP_UART_DMA_RX_BUFFER_SIZE) {
        uart_dma_commit_rx(uart);
    }
}

static void uart_dma_start_tx(struct BSP_UART_Resource *uart)
{
    uint16_t length;

    if ((uart->dma_tx_length != 0U) ||
        (uart->tx_tail == uart->tx_head)) {
        return;
    }
    length = (uart->tx_head > uart->tx_tail)
        ? (uint16_t)(uart->tx_head - uart->tx_tail)
        : (uint16_t)(BSP_UART_TX_BUFFER_SIZE - uart->tx_tail);
    uart->dma_tx_length = length;
    DL_DMA_disableChannel(DMA, uart->dma_tx_channel);
    DL_DMA_clearInterruptStatus(
        DMA, dma_channel_interrupt(uart->dma_tx_channel));
    DL_DMA_setSrcAddr(DMA, uart->dma_tx_channel,
        (uint32_t)&uart->tx_buffer[uart->tx_tail]);
    DL_DMA_setDestAddr(DMA, uart->dma_tx_channel,
        (uint32_t)&uart->instance->TXDATA);
    DL_DMA_setTransferSize(DMA, uart->dma_tx_channel, length);
    DL_DMA_enableChannel(DMA, uart->dma_tx_channel);
}

static void uart_dma_complete_tx(struct BSP_UART_Resource *uart)
{
    const uint16_t completed = uart->dma_tx_length;

    if (completed == 0U) {
        increment_saturating(&uart->dma_error_count);
        uart->sticky_errors |= BSP_UART_RX_ERROR_DMA;
        return;
    }
    uart->tx_tail = (uint16_t)((uart->tx_tail + completed) %
        BSP_UART_TX_BUFFER_SIZE);
    uart->dma_tx_length = 0U;
    uart_dma_start_tx(uart);
}

static void uart_dma_service_tx(struct BSP_UART_Resource *uart)
{
    if ((uart == NULL) || !uart->uses_tx_dma ||
        (uart->dma_tx_length == 0U) ||
        (DL_DMA_getTransferSize(DMA, uart->dma_tx_channel) != 0U)) {
        return;
    }

    /* Do not rely exclusively on the DMA completion IRQ.  If the channel has
     * reached zero but the IRQ has not advanced the ring yet, consume that
     * completion here before testing free space.  Clear the channel event
     * first so a delayed IRQ cannot complete the next segment by mistake. */
    DL_DMA_disableChannel(DMA, uart->dma_tx_channel);
    DL_DMA_clearInterruptStatus(
        DMA, dma_channel_interrupt(uart->dma_tx_channel));
    uart_dma_complete_tx(uart);
}

static void uart_dma_recover_error(struct BSP_UART_Resource *uart)
{
    if (uart->uses_rx_dma) {
        DL_DMA_disableChannel(DMA, uart->dma_rx_channel);
        uart_dma_start_rx(uart);
    }
    if (uart->uses_tx_dma) {
        DL_DMA_disableChannel(DMA, uart->dma_tx_channel);
        DL_DMA_clearInterruptStatus(
            DMA, dma_channel_interrupt(uart->dma_tx_channel));
        /* The exact failing byte is not recoverable after a DMA bus error.
         * Drop the in-flight TX frame and release the queue. */
        uart->tx_tail = uart->tx_head;
        uart->dma_tx_length = 0U;
    }
}

static void uart_fill_tx_fifo(struct BSP_UART_Resource *uart)
{
    while ((uart->tx_tail != uart->tx_head) &&
        !DL_UART_Main_isTXFIFOFull(uart->instance)) {
        DL_UART_Main_transmitData(
            uart->instance, uart->tx_buffer[uart->tx_tail]);
        uart->tx_tail = (uint16_t)((uart->tx_tail + 1U) %
            BSP_UART_TX_BUFFER_SIZE);
    }
    if (uart->tx_tail == uart->tx_head) {
        DL_UART_Main_disableInterrupt(
            uart->instance, DL_UART_INTERRUPT_TX);
    }
}

static void uart_irq(struct BSP_UART_Resource *uart)
{
    DL_UART_IIDX pending;

    do {
        pending = DL_UART_Main_getPendingInterrupt(uart->instance);
        switch (pending) {
        case DL_UART_IIDX_TX:
            uart_fill_tx_fifo(uart);
            break;
        case DL_UART_IIDX_RX:
            if (!uart->uses_rx_dma) {
                uart_drain_fifo(uart);
            }
            break;
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            if (uart->uses_rx_dma) {
                uart_dma_commit_rx(uart);
            }
            else {
                uart_drain_fifo(uart);
            }
            break;
        case DL_UART_IIDX_OVERRUN_ERROR:
            increment_saturating(&uart->hardware_overrun_count);
            uart->sticky_errors |= BSP_UART_RX_ERROR_HARDWARE_OVERRUN;
            break;
        case DL_UART_IIDX_FRAMING_ERROR:
            increment_saturating(&uart->framing_error_count);
            uart->sticky_errors |= BSP_UART_RX_ERROR_FRAMING;
            break;
        case DL_UART_IIDX_PARITY_ERROR:
            increment_saturating(&uart->parity_error_count);
            uart->sticky_errors |= BSP_UART_RX_ERROR_PARITY;
            break;
        default:
            break;
        }
    } while (pending != DL_UART_IIDX_NO_INTERRUPT);
    if (!uart->uses_rx_dma) {
        uart_drain_fifo(uart);
    }
}

static uint32_t wait_budget(uint32_t timeout_ms)
{
    if (timeout_ms > (UINT32_MAX / BSP_UART_WAIT_STEPS_MS)) {
        return UINT32_MAX;
    }
    return timeout_ms * BSP_UART_WAIT_STEPS_MS;
}

static bool wait_step(uint32_t *remaining)
{
    if (*remaining == 0U) {
        return false;
    }
    --(*remaining);
    DL_Common_delayCycles(BSP_RESOURCE_CPU_CLOCK_HZ / 100000U);
    return true;
}

void BSP_UART_Init(void)
{
    const uint32_t error_interrupts = DL_UART_INTERRUPT_OVERRUN_ERROR |
        DL_UART_INTERRUPT_FRAMING_ERROR |
        DL_UART_INTERRUPT_PARITY_ERROR;
    const uint32_t fifo_interrupts = DL_UART_INTERRUPT_RX |
        error_interrupts;
    const uint32_t dma_interrupts = DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |
        error_interrupts;
    const uint32_t managed_interrupts = fifo_interrupts | dma_interrupts |
        DL_UART_INTERRUPT_TX;

    for (size_t index = 0U; index < BSP_UART_COUNT; ++index) {
        /* SysConfig initializes and enables each UART before BSP_Init runs.
         * Discard startup FIFO noise and clear peripheral-side pending flags
         * before exposing the IRQ to NVIC. */
        DL_UART_Main_disableInterrupt(
            g_uart[index].instance, managed_interrupts);
        while (!DL_UART_Main_isRXFIFOEmpty(g_uart[index].instance)) {
            (void)DL_UART_Main_receiveData(g_uart[index].instance);
        }
        DL_UART_Main_clearInterruptStatus(
            g_uart[index].instance, managed_interrupts);
        g_uart[index].rx_head = 0U;
        g_uart[index].rx_tail = 0U;
        g_uart[index].tx_head = 0U;
        g_uart[index].tx_tail = 0U;
        g_uart[index].dma_tx_length = 0U;
        g_uart[index].software_overflow_count = 0U;
        g_uart[index].hardware_overrun_count = 0U;
        g_uart[index].framing_error_count = 0U;
        g_uart[index].parity_error_count = 0U;
        g_uart[index].dma_error_count = 0U;
        g_uart[index].sticky_errors = BSP_UART_RX_ERROR_NONE;
        DL_UART_Main_enableInterrupt(g_uart[index].instance,
            g_uart[index].uses_rx_dma ? dma_interrupts : fifo_interrupts);
        NVIC_ClearPendingIRQ(g_uart[index].irq);
        NVIC_EnableIRQ(g_uart[index].irq);
        if (g_uart[index].uses_rx_dma) {
            uart_dma_start_rx(&g_uart[index]);
        }
    }
    NVIC_ClearPendingIRQ(BSP_RESOURCE_DMA_IRQ);
    NVIC_EnableIRQ(BSP_RESOURCE_DMA_IRQ);
}

size_t BSP_UART_Count(void)
{
    return BSP_UART_COUNT;
}

BSP_UART_Handle BSP_UART_GetHandle(size_t index)
{
    return (index < BSP_UART_COUNT) ? &g_uart[index] : NULL;
}

bool BSP_UART_GetInfo(BSP_UART_Handle uart, BSP_UART_Info *info)
{
    struct BSP_UART_Resource *resource = uart_from_handle(uart);

    if ((resource == NULL) || (info == NULL)) {
        return false;
    }
    *info = (BSP_UART_Info){
        .baud_rate = resource->baud_rate,
        .supports_tx_dma = resource->uses_tx_dma,
        .supports_rx_dma = resource->uses_rx_dma,
        .supports_buffered_tx = true,
        .supports_buffered_rx = true,
        .tx_buffer_capacity = BSP_UART_TX_CAPACITY,
        .rx_buffer_capacity = BSP_UART_RX_CAPACITY,
    };
    return true;
}

BSP_UART_Result BSP_UART_WriteBlocking(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    size_t *transferred)
{
    struct BSP_UART_Resource *resource = uart_from_handle(uart);
    uint32_t remaining = wait_budget(timeout_ms);

    if (transferred != NULL) {
        *transferred = 0U;
    }
    if ((resource == NULL) || (transferred == NULL) ||
        ((data == NULL) && (length != 0U))) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    if ((resource->tx_head != resource->tx_tail) ||
        DL_UART_Main_isBusy(resource->instance)) {
        return BSP_UART_RESULT_BUSY;
    }
    while (*transferred < length) {
        while (DL_UART_Main_isTXFIFOFull(resource->instance)) {
            if (!wait_step(&remaining)) {
                return BSP_UART_RESULT_TIMEOUT;
            }
        }
        DL_UART_Main_transmitData(
            resource->instance, data[*transferred]);
        ++(*transferred);
    }
    return BSP_UART_RESULT_OK;
}

BSP_UART_Result BSP_UART_WriteBuffered(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length)
{
    struct BSP_UART_Resource *resource = uart_from_handle(uart);
    size_t occupied;
    size_t free_bytes;
    uint32_t primask;

    if ((resource == NULL) || ((data == NULL) && (length != 0U)) ||
        (length > BSP_UART_TX_CAPACITY)) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (resource->uses_tx_dma) {
        uart_dma_service_tx(resource);
    }
    else {
        /* Task-side fallback: if a TX interrupt was delayed or lost, refill
         * the hardware FIFO here so the software ring cannot stay wedged. */
        uart_fill_tx_fifo(resource);
    }
    occupied = (resource->tx_head + BSP_UART_TX_BUFFER_SIZE -
        resource->tx_tail) % BSP_UART_TX_BUFFER_SIZE;
    free_bytes = BSP_UART_TX_CAPACITY - occupied;
    if (length > free_bytes) {
        if (primask == 0U) {
            __enable_irq();
        }
        return BSP_UART_RESULT_BUSY;
    }
    for (size_t index = 0U; index < length; ++index) {
        resource->tx_buffer[resource->tx_head] = data[index];
        resource->tx_head = (uint16_t)((resource->tx_head + 1U) %
            BSP_UART_TX_BUFFER_SIZE);
    }
    if (resource->uses_tx_dma) {
        uart_dma_start_tx(resource);
    }
    else {
        uart_fill_tx_fifo(resource);
        if (resource->tx_tail != resource->tx_head) {
            DL_UART_Main_enableInterrupt(
                resource->instance, DL_UART_INTERRUPT_TX);
        }
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return BSP_UART_RESULT_OK;
}

BSP_UART_Result BSP_UART_ReadBlocking(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    size_t *transferred)
{
    struct BSP_UART_Resource *resource = uart_from_handle(uart);
    uint32_t remaining = wait_budget(timeout_ms);

    if (transferred != NULL) {
        *transferred = 0U;
    }
    if ((resource == NULL) || (transferred == NULL) ||
        ((data == NULL) && (length != 0U))) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    while (*transferred < length) {
        if (resource->rx_tail != resource->rx_head) {
            data[*transferred] = resource->rx_buffer[resource->rx_tail];
            resource->rx_tail = (uint16_t)((resource->rx_tail + 1U) %
                BSP_UART_RX_BUFFER_SIZE);
            ++(*transferred);
        }
        else if (!wait_step(&remaining)) {
            return BSP_UART_RESULT_TIMEOUT;
        }
    }
    return BSP_UART_RESULT_OK;
}

BSP_UART_Result BSP_UART_ReadAvailable(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t capacity,
    size_t *transferred)
{
    struct BSP_UART_Resource *resource = uart_from_handle(uart);
    uint32_t primask;

    if (transferred != NULL) {
        *transferred = 0U;
    }
    if ((resource == NULL) || (transferred == NULL) ||
        ((data == NULL) && (capacity != 0U))) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (resource->uses_rx_dma) {
        uart_dma_flush_partial_rx(resource);
    }
    else {
        /* Task-side fallback mirrors the RX ISR and recovers even if an RX
         * edge was missed while interrupts were briefly masked. */
        uart_drain_fifo(resource);
    }
    if (primask == 0U) {
        __enable_irq();
    }
    while ((*transferred < capacity) &&
        (resource->rx_tail != resource->rx_head)) {
        data[*transferred] = resource->rx_buffer[resource->rx_tail];
        resource->rx_tail = (uint16_t)((resource->rx_tail + 1U) %
            BSP_UART_RX_BUFFER_SIZE);
        ++(*transferred);
    }
    return BSP_UART_RESULT_OK;
}

bool BSP_UART_GetRxStatus(BSP_UART_Handle uart, BSP_UART_RxStatus *status)
{
    struct BSP_UART_Resource *resource = uart_from_handle(uart);
    BSP_UART_RxStatus next;
    uint32_t primask;

    if ((resource == NULL) || (status == NULL)) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    next = (BSP_UART_RxStatus){
        .software_overflow_count = resource->software_overflow_count,
        .hardware_overrun_count = resource->hardware_overrun_count,
        .framing_error_count = resource->framing_error_count,
        .parity_error_count = resource->parity_error_count,
        .dma_error_count = resource->dma_error_count,
        .sticky_errors = resource->sticky_errors,
        .buffered_bytes = (resource->rx_head + BSP_UART_RX_BUFFER_SIZE -
            resource->rx_tail) % BSP_UART_RX_BUFFER_SIZE,
        .buffer_capacity = BSP_UART_RX_CAPACITY,
    };
    if (primask == 0U) {
        __enable_irq();
    }
    *status = next;
    return true;
}

bool BSP_UART_ClearRxStatus(BSP_UART_Handle uart)
{
    struct BSP_UART_Resource *resource = uart_from_handle(uart);
    uint32_t primask;

    if (resource == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    resource->software_overflow_count = 0U;
    resource->hardware_overrun_count = 0U;
    resource->framing_error_count = 0U;
    resource->parity_error_count = 0U;
    resource->dma_error_count = 0U;
    resource->sticky_errors = BSP_UART_RX_ERROR_NONE;
    if (resource->uses_rx_dma) {
        /* Discard only the uncommitted DMA staging bytes belonging to the
         * damaged frame. Bytes already in the public RX ring are retained. */
        uart_dma_start_rx(resource);
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

bool BSP_UART_Recover(BSP_UART_Handle uart)
{
    struct BSP_UART_Resource *resource = uart_from_handle(uart);
    const uint32_t error_interrupts = DL_UART_INTERRUPT_OVERRUN_ERROR |
        DL_UART_INTERRUPT_FRAMING_ERROR |
        DL_UART_INTERRUPT_PARITY_ERROR;
    const uint32_t rx_interrupts = resource != NULL && resource->uses_rx_dma ?
        (DL_UART_INTERRUPT_RX_TIMEOUT_ERROR | error_interrupts) :
        (DL_UART_INTERRUPT_RX | error_interrupts);
    const uint32_t managed_interrupts = rx_interrupts |
        DL_UART_INTERRUPT_TX;
    uint32_t primask;
    bool fifo_enabled;

    if (resource == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    NVIC_DisableIRQ(resource->irq);
    DL_UART_Main_disableInterrupt(resource->instance, managed_interrupts);
    if (resource->uses_rx_dma) {
        DL_DMA_disableChannel(DMA, resource->dma_rx_channel);
        DL_DMA_clearInterruptStatus(
            DMA, dma_channel_interrupt(resource->dma_rx_channel));
    }
    if (resource->uses_tx_dma) {
        DL_DMA_disableChannel(DMA, resource->dma_tx_channel);
        DL_DMA_clearInterruptStatus(
            DMA, dma_channel_interrupt(resource->dma_tx_channel));
    }

    /* Disable immediately instead of waiting for BUSY to clear: this path is
     * specifically the escape hatch for a wedged transmitter. Toggling FEN
     * flushes both hardware FIFOs while preserving the SysConfig settings. */
    fifo_enabled = DL_UART_Main_isFIFOsEnabled(resource->instance);
    DL_UART_Main_disable(resource->instance);
    if (fifo_enabled) {
        DL_UART_Main_disableFIFOs(resource->instance);
        DL_UART_Main_enableFIFOs(resource->instance);
    }
    while (!DL_UART_Main_isRXFIFOEmpty(resource->instance)) {
        (void)DL_UART_Main_receiveData(resource->instance);
    }

    resource->rx_head = 0U;
    resource->rx_tail = 0U;
    resource->tx_head = 0U;
    resource->tx_tail = 0U;
    resource->dma_tx_length = 0U;
    resource->software_overflow_count = 0U;
    resource->hardware_overrun_count = 0U;
    resource->framing_error_count = 0U;
    resource->parity_error_count = 0U;
    resource->dma_error_count = 0U;
    resource->sticky_errors = BSP_UART_RX_ERROR_NONE;
    DL_UART_Main_clearInterruptStatus(
        resource->instance, managed_interrupts);
    DL_UART_Main_enable(resource->instance);
    DL_UART_Main_enableInterrupt(resource->instance, rx_interrupts);
    if (resource->uses_rx_dma) {
        uart_dma_start_rx(resource);
    }
    NVIC_ClearPendingIRQ(resource->irq);
    NVIC_EnableIRQ(resource->irq);
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

BSP_UART_Result BSP_UART_StartWriteDMA(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    BSP_UART_TransferCallback callback,
    void *context)
{
    (void)data;
    (void)length;
    (void)timeout_ms;
    (void)callback;
    (void)context;
    return (uart_from_handle(uart) == NULL)
        ? BSP_UART_RESULT_INVALID_ARGUMENT
        : BSP_UART_RESULT_UNSUPPORTED;
}

BSP_UART_Result BSP_UART_StartReadDMA(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    BSP_UART_TransferCallback callback,
    void *context)
{
    (void)data;
    (void)length;
    (void)timeout_ms;
    (void)callback;
    (void)context;
    return (uart_from_handle(uart) == NULL)
        ? BSP_UART_RESULT_INVALID_ARGUMENT
        : BSP_UART_RESULT_UNSUPPORTED;
}

bool BSP_UART_IsTxBusy(BSP_UART_Handle uart)
{
    struct BSP_UART_Resource *resource = uart_from_handle(uart);
    uint32_t primask;

    if (resource == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    uart_dma_service_tx(resource);
    if (primask == 0U) {
        __enable_irq();
    }

    return (resource->tx_head != resource->tx_tail) ||
        DL_UART_Main_isBusy(resource->instance);
}

bool BSP_UART_IsRxBusy(BSP_UART_Handle uart)
{
    return (uart_from_handle(uart) != NULL) && false;
}

void DMA_IRQHandler(void)
{
    DL_DMA_EVENT_IIDX pending;

    do {
        pending = DL_DMA_getPendingInterrupt(DMA);
        switch (pending) {
        case DL_DMA_EVENT_IIDX_DMACH2:
            uart_dma_commit_rx(&g_uart[2]);
            break;
        case DL_DMA_EVENT_IIDX_ADDR_ERROR:
        case DL_DMA_EVENT_IIDX_DATA_ERROR:
            if (g_uart[0].uses_rx_dma || g_uart[0].uses_tx_dma) {
                increment_saturating(&g_uart[0].dma_error_count);
                g_uart[0].sticky_errors |= BSP_UART_RX_ERROR_DMA;
                uart_dma_recover_error(&g_uart[0]);
            }
            if (g_uart[2].uses_rx_dma || g_uart[2].uses_tx_dma) {
                increment_saturating(&g_uart[2].dma_error_count);
                g_uart[2].sticky_errors |= BSP_UART_RX_ERROR_DMA;
                uart_dma_recover_error(&g_uart[2]);
            }
            break;
        default:
            break;
        }
    } while (pending != DL_DMA_EVENT_IIDX_NO_INTR);
}

void UART0_IRQHandler(void)
{
    uart_irq(&g_uart[1]);
}

void UART1_IRQHandler(void)
{
    uart_irq(&g_uart[0]);
}

void UART4_IRQHandler(void)
{
    uart_irq(&g_uart[2]);
}
