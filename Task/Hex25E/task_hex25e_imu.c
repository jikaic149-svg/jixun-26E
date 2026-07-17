#include "task_hex25e_imu.h"
#include "hex25e_config.h"

#include "device_serial.h"

#include <stddef.h>

typedef struct {
    Device_Serial serial;
    Device_JY61P parser;
    bool initialized;
} Task_Hex25E_IMU_Context;

static Task_Hex25E_IMU_Context g_imu;

bool Task_Hex25E_IMU_Init(void)
{
    const Device_Serial_Config serial_config = {
        .transport_index = HEX25E_CONFIG_IMU_TRANSPORT_INDEX,
        .expected_baud_rate = HEX25E_CONFIG_IMU_BAUD_RATE,
        .minimum_tx_capacity = 0U,
        .minimum_rx_capacity = 1U,
        .require_buffered_tx = false,
        .require_buffered_rx = true,
    };
    Task_Hex25E_IMU_Context next = {0};

    if (!Device_Serial_Init(&next.serial, &serial_config) ||
        !Device_JY61P_Init(&next.parser, NULL)) {
        g_imu = (Task_Hex25E_IMU_Context){0};
        return false;
    }
    next.initialized = true;
    g_imu = next;
    return true;
}

bool Task_Hex25E_IMU_Update(uint32_t now_ms)
{
    uint8_t bytes[HEX25E_CONFIG_IMU_RX_CHUNK_SIZE];

    if (!g_imu.initialized) {
        return false;
    }
    for (size_t chunk = 0U;
         chunk < HEX25E_CONFIG_IMU_MAX_CHUNKS_PER_UPDATE; ++chunk) {
        size_t received = 0U;

        if (Device_Serial_ReadAvailable(
                &g_imu.serial, bytes, sizeof(bytes), &received) !=
            DEVICE_SERIAL_RESULT_OK) {
            return false;
        }
        for (size_t index = 0U; index < received; ++index) {
            (void)Device_JY61P_FeedByte(
                &g_imu.parser, bytes[index], now_ms);
        }
        if (received < sizeof(bytes)) {
            break;
        }
    }
    Device_JY61P_UpdateOnline(&g_imu.parser, now_ms);
    return true;
}

bool Task_Hex25E_IMU_GetSnapshot(Device_JY61P_Snapshot *snapshot)
{
    return g_imu.initialized &&
        Device_JY61P_GetSnapshot(&g_imu.parser, snapshot);
}
