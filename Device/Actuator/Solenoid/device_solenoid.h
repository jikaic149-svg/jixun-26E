#ifndef DEVICE_SOLENOID_H
#define DEVICE_SOLENOID_H

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    DEVICE_SOLENOID_RESULT_OK = 0,
    DEVICE_SOLENOID_RESULT_INVALID_ARGUMENT,
    DEVICE_SOLENOID_RESULT_BACKEND_ERROR,
} Device_Solenoid_Result;

typedef struct {
    size_t backend_index;
} Device_Solenoid_Config;

typedef struct {
    Device_Solenoid_Config config;
    bool on;
    bool initialized;
} Device_Solenoid;

bool Device_Solenoid_DefaultConfig(
    size_t index, Device_Solenoid_Config *config);
/* Init always drives the output low before reporting success. */
bool Device_Solenoid_Init(
    Device_Solenoid *solenoid, const Device_Solenoid_Config *config);
Device_Solenoid_Result Device_Solenoid_Set(
    Device_Solenoid *solenoid, bool on);
bool Device_Solenoid_IsOn(const Device_Solenoid *solenoid);

#endif
