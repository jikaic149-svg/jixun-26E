#include "device_buzzer.h"
#include "../BSP/bsp_buzzer.h"

void Device_Buzzer_Set(bool state) {
    BSP_Buzzer_Set(state);
}
