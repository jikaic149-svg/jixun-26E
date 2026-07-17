#ifndef DEVICE_JY61P_DEVICE_JY61P_H
#define DEVICE_JY61P_DEVICE_JY61P_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float acceleration_mps2[3];
    float angular_rate_dps[3];
    float euler_deg[3];
    float yaw_zeroed_deg;
    uint32_t last_valid_frame_ms;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    bool has_acceleration;
    bool has_angular_rate;
    bool has_euler;
    bool online;
} Device_JY61P_Snapshot;

typedef struct {
    uint32_t offline_timeout_ms;
} Device_JY61P_Config;

typedef struct {
    Device_JY61P_Config config;
    Device_JY61P_Snapshot snapshot;
    uint8_t frame[11];
    uint8_t frame_index;
    float yaw_offset_deg;
    bool yaw_offset_ready;
    bool initialized;
} Device_JY61P;

Device_JY61P_Config Device_JY61P_DefaultConfig(void);
bool Device_JY61P_Init(
    Device_JY61P *device, const Device_JY61P_Config *config);

/* Feed bytes drained from the owning UART/SPSC ring in main/task context.
 * Returns true only when this byte completes a checksum-valid 0x51/52/53
 * frame. No ISR, UART handle, Broker, or RTOS object is owned here. */
bool Device_JY61P_FeedByte(
    Device_JY61P *device, uint8_t byte, uint32_t now_ms);
void Device_JY61P_UpdateOnline(Device_JY61P *device, uint32_t now_ms);
bool Device_JY61P_TareYaw(Device_JY61P *device);
bool Device_JY61P_GetSnapshot(
    const Device_JY61P *device, Device_JY61P_Snapshot *snapshot);

#endif
