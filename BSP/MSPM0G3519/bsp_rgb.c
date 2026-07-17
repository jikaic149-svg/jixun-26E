#include "bsp_rgb.h"

#include <stdbool.h>

#include "bsp_resource_map.h"

#define RGB_BITS_PER_PIXEL       24U
#define RGB_COMPARE_ZERO         75U
#define RGB_COMPARE_ONE          50U
#define RGB_COMPARE_IDLE         99U
#define RGB_RESET_DELAY_CYCLES   (BSP_RESOURCE_CPU_CLOCK_HZ / 10000U)
#define RGB_WAIT_LIMIT           (BSP_RESOURCE_CPU_CLOCK_HZ / 1000U)

volatile uint8_t g_BSP_RGB_LastGRB[3] = {0U, 0U, 0U};
volatile uint32_t g_BSP_RGB_FrameCount = 0U;

static BSP_RGB_Pixel g_rgb_pixels[BSP_RESOURCE_RGB_COUNT];
static volatile uint8_t g_rgb_compare[RGB_BITS_PER_PIXEL];
static volatile uint8_t g_rgb_next_bit = 0U;
static volatile bool g_rgb_frame_done = true;

static void rgb_prepare_frame(const BSP_RGB_Pixel *pixel)
{
    const uint8_t frame[3] = {
        pixel->green,
        pixel->red,
        pixel->blue,
    };
    uint32_t compare_index = 0U;

    g_BSP_RGB_LastGRB[0] = frame[0];
    g_BSP_RGB_LastGRB[1] = frame[1];
    g_BSP_RGB_LastGRB[2] = frame[2];

    for (uint32_t byte_index = 0U; byte_index < 3U; ++byte_index) {
        for (uint32_t bit_mask = 0x80U; bit_mask != 0U; bit_mask >>= 1U) {
            g_rgb_compare[compare_index] =
                ((frame[byte_index] & bit_mask) != 0U)
                    ? RGB_COMPARE_ONE
                    : RGB_COMPARE_ZERO;
            ++compare_index;
        }
    }
}

static void rgb_stop_output(void)
{
    DL_Timer_overrideCCPOut(BSP_RESOURCE_RGB_TIMER, DL_TIMER_FORCE_OUT_LOW,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED, BSP_RESOURCE_RGB_CC_INDEX);
    DL_TimerG_stopCounter(BSP_RESOURCE_RGB_TIMER);
    DL_TimerG_disableInterrupt(
        BSP_RESOURCE_RGB_TIMER, DL_TIMER_INTERRUPT_LOAD_EVENT);
    NVIC_DisableIRQ(BSP_RESOURCE_RGB_TIMER_IRQ);
    g_rgb_frame_done = true;
}

static bool rgb_send_frame(void)
{
    uint32_t wait_limit = RGB_WAIT_LIMIT;

    rgb_prepare_frame(&g_rgb_pixels[0]);

    NVIC_DisableIRQ(BSP_RESOURCE_RGB_TIMER_IRQ);
    DL_TimerG_stopCounter(BSP_RESOURCE_RGB_TIMER);
    DL_TimerG_disableInterrupt(
        BSP_RESOURCE_RGB_TIMER, DL_TIMER_INTERRUPT_LOAD_EVENT);
    DL_TimerG_clearInterruptStatus(
        BSP_RESOURCE_RGB_TIMER, DL_TIMER_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(BSP_RESOURCE_RGB_TIMER_IRQ);

    DL_TimerG_setCaptCompUpdateMethod(
        BSP_RESOURCE_RGB_TIMER, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        BSP_RESOURCE_RGB_CC_INDEX);
    DL_TimerG_setCaptureCompareValue(
        BSP_RESOURCE_RGB_TIMER, g_rgb_compare[0],
        BSP_RESOURCE_RGB_CC_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(
        BSP_RESOURCE_RGB_TIMER, DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT,
        BSP_RESOURCE_RGB_CC_INDEX);

    g_rgb_next_bit = 1U;
    g_rgb_frame_done = false;

    DL_Timer_overrideCCPOut(
        BSP_RESOURCE_RGB_TIMER, DL_TIMER_FORCE_OUT_DISABLED,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED, BSP_RESOURCE_RGB_CC_INDEX);
    DL_TimerG_enableInterrupt(
        BSP_RESOURCE_RGB_TIMER, DL_TIMER_INTERRUPT_LOAD_EVENT);
    NVIC_EnableIRQ(BSP_RESOURCE_RGB_TIMER_IRQ);
    DL_TimerG_startCounter(BSP_RESOURCE_RGB_TIMER);

    while (!g_rgb_frame_done && wait_limit > 0U) {
        __NOP();
        --wait_limit;
    }

    if (!g_rgb_frame_done) {
        rgb_stop_output();
        return false;
    }

    /* The latch/reset interval belongs to the BSP transaction. Callers may
     * invoke BSP_RGB_Show() back-to-back without adding an external delay. */
    DL_Common_delayCycles(RGB_RESET_DELAY_CYCLES);
    ++g_BSP_RGB_FrameCount;
    return true;
}

void BSP_RESOURCE_RGB_IRQ_HANDLER(void)
{
    if (DL_TimerG_getPendingInterrupt(BSP_RESOURCE_RGB_TIMER) !=
        DL_TIMER_IIDX_LOAD) {
        return;
    }

    if (g_rgb_next_bit < RGB_BITS_PER_PIXEL) {
        DL_TimerG_setCaptureCompareValue(
            BSP_RESOURCE_RGB_TIMER, g_rgb_compare[g_rgb_next_bit],
            BSP_RESOURCE_RGB_CC_INDEX);
        ++g_rgb_next_bit;
    } else if (g_rgb_next_bit == RGB_BITS_PER_PIXEL) {
        DL_TimerG_setCaptureCompareValue(
            BSP_RESOURCE_RGB_TIMER, RGB_COMPARE_IDLE,
            BSP_RESOURCE_RGB_CC_INDEX);
        ++g_rgb_next_bit;
    } else {
        rgb_stop_output();
    }
}

void BSP_RGB_Init(void)
{
    g_rgb_pixels[0] = (BSP_RGB_Pixel) {0U, 0U, 0U};

    DL_TimerG_stopCounter(BSP_RESOURCE_RGB_TIMER);
    DL_Timer_overrideCCPOut(BSP_RESOURCE_RGB_TIMER, DL_TIMER_FORCE_OUT_LOW,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED, BSP_RESOURCE_RGB_CC_INDEX);
    DL_Common_delayCycles(RGB_RESET_DELAY_CYCLES);
}

size_t BSP_RGB_Count(void)
{
    return BSP_RESOURCE_RGB_COUNT;
}

bool BSP_RGB_SetPixel(size_t index, BSP_RGB_Pixel color)
{
    if (index >= BSP_RESOURCE_RGB_COUNT) {
        return false;
    }

    g_rgb_pixels[index] = color;
    return true;
}

bool BSP_RGB_Show(void)
{
    return rgb_send_frame();
}
