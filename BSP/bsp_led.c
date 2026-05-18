#include "bsp_led.h"
#include "ti_msp_dl_config.h"

void BSP_LED_Toggle(uint8_t led_id) {
    if (led_id == 1) {
        DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_BLUE_PIN);
    } else if (led_id == 2) {
        DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_ORANGE_PIN);
    }
}

void BSP_LED_Set(uint8_t led_id, bool state) {
    uint32_t pin = 0;
    if (led_id == 1) pin = GPIO_LEDS_USER_LED_BLUE_PIN;
    else if (led_id == 2) pin = GPIO_LEDS_USER_LED_ORANGE_PIN;
    else return;

    /* LP_MSPM0G3519 板载 LED 是 active-low：拉低=亮、拉高=灭 */
    if (state) {
        DL_GPIO_clearPins(GPIO_LEDS_PORT, pin);
    } else {
        DL_GPIO_setPins(GPIO_LEDS_PORT, pin);
    }
}
