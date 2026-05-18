#include "bsp_buzzer.h"
#include "ti_msp_dl_config.h"

void BSP_Buzzer_Set(bool state) {
    /* 蜂鸣器接 3.3V 和 PA0，低电平触发。
     * PA0拉低(0V) -> 有3.3V压差 -> 响。
     * PA0拉高(3.3V) -> 0V压差 -> 停。 */
    if (state) {
        DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_PIN_BUZZER_PIN); // 响
    } else {
        DL_GPIO_setPins(GPIO_BUZZER_PORT, GPIO_BUZZER_PIN_BUZZER_PIN);   // 停
    }
}
