#include "device_led.h"

void Device_LED_Init(Device_LED_t *led, Device_LED_Init_Config_t *config)
{
    if (led == 0 || config == 0) return;
    
    led->WritePin = config->WritePin;
    led->TogglePin = config->TogglePin;
    led->IsOn = config->InitState;
    
    if (led->WritePin) {
        led->WritePin(led->IsOn);
    }
}

void Device_LED_Set(Device_LED_t *led, bool state)
{
    if (!led) return;
    
    led->IsOn = state;
    if (led->WritePin) {
        led->WritePin(led->IsOn);
    }
}

void Device_LED_Toggle(Device_LED_t *led)
{
    if (!led) return;
    
    led->IsOn = !led->IsOn;
    if (led->TogglePin) {
        led->TogglePin();
    } else if (led->WritePin) {
        led->WritePin(led->IsOn);
    }
}
