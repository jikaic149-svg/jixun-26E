#include "bsp_qei.h"
#include "ti_msp_dl_config.h"

static uint16_t last_count_L = 0;
static uint16_t last_count_R = 0;

void BSP_QEI_Init(void) {
    /* 
     * 编码器引脚分配（遵循 SysConfig 生成配置）：
     * 右侧编码器 (QEI_0 / TIMG9): PHA = PA3, PHB = PA2
     * 左侧编码器 (QEI_1 / TIMG8): PHA = PA1, PHB = PA0
     * 
     * [防冲突提醒]：根据开发板硬件规范，PA0 和 PA1 是 EVM_A 区域的 I2C 预留引脚。
     * 当前我们的工程并没有挂载 I2C 设备（如 OLED 屏幕等），所以引脚不冲突、可安全使用。
     * 如果未来需要加入 I2C，建议将 QEI_1 换到别的引脚。
     */
    DL_TimerG_startCounter(QEI_Left_INST);
    DL_TimerG_startCounter(QEI_Right_INST);
    
    last_count_L = DL_TimerG_getTimerCount(QEI_Left_INST);
    last_count_R = DL_TimerG_getTimerCount(QEI_Right_INST);
}

int16_t BSP_QEI_GetDeltaL(void) {
    // TIMG 的 QEI 计数值是 16 位的 (0 ~ 65535)。
    // 强制类型转换为 int16_t 可以在溢出/下溢时自动得出正确的正负增量。
    uint16_t current = DL_TimerG_getTimerCount(QEI_Left_INST);
    int16_t delta = (int16_t)(current - last_count_L);
    last_count_L = current;
    
    // 若电机正转但增量为负，直接在此处 return -delta; 翻转极性
    return delta; 
}

int16_t BSP_QEI_GetDeltaR(void) {
    uint16_t current = DL_TimerG_getTimerCount(QEI_Right_INST);
    int16_t delta = (int16_t)(current - last_count_R);
    last_count_R = current;
    
    // 若电机正转但增量为负，直接在此处 return -delta; 翻转极性
    return -delta;
}
