#include "device_jy61p.h"

#include <limits.h>
#include <stddef.h>

#define DEVICE_JY61P_HEADER 0x55U
#define DEVICE_JY61P_ACCELERATION 0x51U
#define DEVICE_JY61P_ANGULAR_RATE 0x52U
#define DEVICE_JY61P_ANGLE 0x53U

static void Device_JY61P_Increment(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        ++(*value);
    }
}

static int16_t Device_JY61P_ReadI16(const uint8_t *bytes)
{
    const uint16_t raw = (uint16_t)(
        (uint16_t)bytes[0] |
        (uint16_t)((uint16_t)bytes[1] << 8U));

    if (raw <= (uint16_t)INT16_MAX) {
        return (int16_t)raw;
    }
    return (int16_t)((int32_t)raw - INT32_C(65536));
}

static float Device_JY61P_NormalizeAngle(float angle)
{
    while (angle >= 180.0F) {
        angle -= 360.0F;
    }
    while (angle < -180.0F) {
        angle += 360.0F;
    }
    return angle;
}

static bool Device_JY61P_ValidType(uint8_t type)
{
    return (type == DEVICE_JY61P_ACCELERATION) ||
           (type == DEVICE_JY61P_ANGULAR_RATE) ||
           (type == DEVICE_JY61P_ANGLE);
}

static bool Device_JY61P_ChecksumValid(const uint8_t *frame)
{
    uint8_t sum = 0U;
    uint8_t index;

    for (index = 0U; index < 10U; ++index) {
        sum = (uint8_t)(sum + frame[index]);
    }
    return sum == frame[10];
}

static void Device_JY61P_DecodeVector(
    const uint8_t *frame, float scale, float *values)
{
    uint8_t axis;

    for (axis = 0U; axis < 3U; ++axis) {
        values[axis] =
            ((float)Device_JY61P_ReadI16(&frame[2U + (axis * 2U)]) /
             32768.0F) * scale;
    }
}

static void Device_JY61P_Decode(
    Device_JY61P *device, uint32_t now_ms)
{
    const uint8_t type = device->frame[1];

    if (type == DEVICE_JY61P_ACCELERATION) {
        Device_JY61P_DecodeVector(
            device->frame, 16.0F * 9.80665F,
            device->snapshot.acceleration_mps2);
        device->snapshot.has_acceleration = true;
    }
    else if (type == DEVICE_JY61P_ANGULAR_RATE) {
        Device_JY61P_DecodeVector(
            device->frame, 2000.0F,
            device->snapshot.angular_rate_dps);
        device->snapshot.has_angular_rate = true;
    }
    else {
        Device_JY61P_DecodeVector(
            device->frame, 180.0F, device->snapshot.euler_deg);
        if (!device->yaw_offset_ready) {
            device->yaw_offset_deg = device->snapshot.euler_deg[2];
            device->yaw_offset_ready = true;
        }
        device->snapshot.yaw_zeroed_deg = Device_JY61P_NormalizeAngle(
            device->snapshot.euler_deg[2] - device->yaw_offset_deg);
        device->snapshot.has_euler = true;
    }

    device->snapshot.last_valid_frame_ms = now_ms;
    device->snapshot.online = true;
    Device_JY61P_Increment(&device->snapshot.valid_frame_count);
}

Device_JY61P_Config Device_JY61P_DefaultConfig(void)
{
    const Device_JY61P_Config config = {
        .offline_timeout_ms = 500U,
    };
    return config;
}

bool Device_JY61P_Init(
    Device_JY61P *device, const Device_JY61P_Config *config)
{
    const Device_JY61P_Config selected =
        (config == NULL) ? Device_JY61P_DefaultConfig() : *config;

    if (device != NULL) {
        *device = (Device_JY61P){0};
    }
    if ((device == NULL) || (selected.offline_timeout_ms == 0U) ||
        (selected.offline_timeout_ms > (UINT32_MAX / 2U))) {
        return false;
    }
    device->config = selected;
    device->initialized = true;
    return true;
}

bool Device_JY61P_FeedByte(
    Device_JY61P *device, uint8_t byte, uint32_t now_ms)
{
    if ((device == NULL) || !device->initialized) {
        return false;
    }

    /* Public structs are used for static allocation, but a corrupted parser
     * cursor must never become an out-of-bounds write. */
    if (device->frame_index >= sizeof(device->frame)) {
        device->frame_index = 0U;
    }

    if (device->frame_index == 0U) {
        if (byte == DEVICE_JY61P_HEADER) {
            device->frame[0] = byte;
            device->frame_index = 1U;
        }
        return false;
    }
    if (device->frame_index == 1U) {
        if (Device_JY61P_ValidType(byte)) {
            device->frame[1] = byte;
            device->frame_index = 2U;
        }
        else if (byte != DEVICE_JY61P_HEADER) {
            device->frame_index = 0U;
        }
        return false;
    }

    device->frame[device->frame_index] = byte;
    ++device->frame_index;
    if (device->frame_index < sizeof(device->frame)) {
        return false;
    }
    device->frame_index = 0U;
    if (!Device_JY61P_ChecksumValid(device->frame)) {
        Device_JY61P_Increment(&device->snapshot.checksum_error_count);
        /* If a damaged/short frame consumed the next 0x55 as its checksum,
         * retain that byte as the next candidate header for fast recovery. */
        if (byte == DEVICE_JY61P_HEADER) {
            device->frame[0] = DEVICE_JY61P_HEADER;
            device->frame_index = 1U;
        }
        return false;
    }
    Device_JY61P_Decode(device, now_ms);
    return true;
}

void Device_JY61P_UpdateOnline(Device_JY61P *device, uint32_t now_ms)
{
    if ((device == NULL) || !device->initialized ||
        (device->snapshot.valid_frame_count == 0U)) {
        return;
    }
    device->snapshot.online =
        (uint32_t)(now_ms - device->snapshot.last_valid_frame_ms) <=
        device->config.offline_timeout_ms;
}

bool Device_JY61P_TareYaw(Device_JY61P *device)
{
    if ((device == NULL) || !device->initialized ||
        !device->snapshot.has_euler) {
        return false;
    }
    device->yaw_offset_deg = device->snapshot.euler_deg[2];
    device->yaw_offset_ready = true;
    device->snapshot.yaw_zeroed_deg = 0.0F;
    return true;
}

bool Device_JY61P_GetSnapshot(
    const Device_JY61P *device, Device_JY61P_Snapshot *snapshot)
{
    if ((device == NULL) || !device->initialized || (snapshot == NULL)) {
        return false;
    }
    *snapshot = device->snapshot;
    return true;
}
