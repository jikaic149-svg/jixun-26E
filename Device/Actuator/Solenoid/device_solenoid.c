#include "device_solenoid.h"

#include "bsp_solenoid.h"

#include <stddef.h>

bool Device_Solenoid_DefaultConfig(
    size_t index, Device_Solenoid_Config *config)
{
    if ((config == NULL) || (index >= BSP_Solenoid_Count())) {
        return false;
    }
    *config = (Device_Solenoid_Config){
        .backend_index = index,
    };
    return true;
}

bool Device_Solenoid_Init(
    Device_Solenoid *solenoid, const Device_Solenoid_Config *config)
{
    if (solenoid != NULL) {
        *solenoid = (Device_Solenoid){0};
    }
    if ((solenoid == NULL) || (config == NULL) ||
        (config->backend_index >= BSP_Solenoid_Count()) ||
        !BSP_Solenoid_Set(config->backend_index, false)) {
        return false;
    }
    solenoid->config = *config;
    solenoid->initialized = true;
    return true;
}

Device_Solenoid_Result Device_Solenoid_Set(
    Device_Solenoid *solenoid, bool on)
{
    if ((solenoid == NULL) || !solenoid->initialized) {
        return DEVICE_SOLENOID_RESULT_INVALID_ARGUMENT;
    }
    if (solenoid->on == on) {
        return DEVICE_SOLENOID_RESULT_OK;
    }
    if (!BSP_Solenoid_Set(solenoid->config.backend_index, on)) {
        solenoid->on = false;
        (void)BSP_Solenoid_Set(solenoid->config.backend_index, false);
        return DEVICE_SOLENOID_RESULT_BACKEND_ERROR;
    }
    solenoid->on = on;
    return DEVICE_SOLENOID_RESULT_OK;
}

bool Device_Solenoid_IsOn(const Device_Solenoid *solenoid)
{
    return (solenoid != NULL) && solenoid->initialized && solenoid->on;
}
