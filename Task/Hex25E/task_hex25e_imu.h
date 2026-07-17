#ifndef TASK_HEX25E_IMU_H
#define TASK_HEX25E_IMU_H

#include "device_jy61p.h"

#include <stdbool.h>
#include <stdint.h>

/* Bind the Hex25E JY61P role through the generic Device serial transport. */
bool Task_Hex25E_IMU_Init(void);

/* Drain the transport without blocking, feed complete JY61P frames, and
 * refresh the online flag. Called from the control task, never an ISR. */
bool Task_Hex25E_IMU_Update(uint32_t now_ms);

bool Task_Hex25E_IMU_GetSnapshot(Device_JY61P_Snapshot *snapshot);

#endif
