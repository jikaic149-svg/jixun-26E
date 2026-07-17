#ifndef DEVICE_SENSOR_IMU_ICM45686_DEVICE_ICM45686_H
#define DEVICE_SENSOR_IMU_ICM45686_DEVICE_ICM45686_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "device_imu.h"

#define DEVICE_ICM45686_WHO_AM_I_VALUE UINT8_C(0xE9)
#define DEVICE_ICM45686_SPI_MAX_HZ UINT32_C(24000000)

typedef enum {
    DEVICE_ICM45686_ACCEL_RANGE_32G = 0,
    DEVICE_ICM45686_ACCEL_RANGE_16G,
    DEVICE_ICM45686_ACCEL_RANGE_8G,
    DEVICE_ICM45686_ACCEL_RANGE_4G,
    DEVICE_ICM45686_ACCEL_RANGE_2G,
} Device_ICM45686_AccelRange;

typedef enum {
    DEVICE_ICM45686_GYRO_RANGE_4000_DPS = 0,
    DEVICE_ICM45686_GYRO_RANGE_2000_DPS,
    DEVICE_ICM45686_GYRO_RANGE_1000_DPS,
    DEVICE_ICM45686_GYRO_RANGE_500_DPS,
    DEVICE_ICM45686_GYRO_RANGE_250_DPS,
    DEVICE_ICM45686_GYRO_RANGE_125_DPS,
    DEVICE_ICM45686_GYRO_RANGE_62_5_DPS,
    DEVICE_ICM45686_GYRO_RANGE_31_25_DPS,
    DEVICE_ICM45686_GYRO_RANGE_15_625_DPS,
} Device_ICM45686_GyroRange;

typedef enum {
    DEVICE_ICM45686_ODR_6400_HZ = 0x03,
    DEVICE_ICM45686_ODR_3200_HZ = 0x04,
    DEVICE_ICM45686_ODR_1600_HZ = 0x05,
    DEVICE_ICM45686_ODR_800_HZ = 0x06,
    DEVICE_ICM45686_ODR_400_HZ = 0x07,
    DEVICE_ICM45686_ODR_200_HZ = 0x08,
    DEVICE_ICM45686_ODR_100_HZ = 0x09,
    DEVICE_ICM45686_ODR_50_HZ = 0x0A,
    DEVICE_ICM45686_ODR_25_HZ = 0x0B,
    DEVICE_ICM45686_ODR_12_5_HZ = 0x0C,
    DEVICE_ICM45686_ODR_6_25_HZ = 0x0D,
    DEVICE_ICM45686_ODR_3_125_HZ = 0x0E,
    DEVICE_ICM45686_ODR_1_5625_HZ = 0x0F,
} Device_ICM45686_Odr;

typedef enum {
    DEVICE_ICM45686_DATA_LITTLE_ENDIAN = 0,
    DEVICE_ICM45686_DATA_BIG_ENDIAN,
} Device_ICM45686_DataEndianness;

typedef bool (*Device_ICM45686_ReadRegisters)(
    void *context,
    uint8_t first_register,
    uint8_t *bytes,
    size_t length);
typedef bool (*Device_ICM45686_WriteRegister)(
    void *context,
    uint8_t register_address,
    uint8_t value);
typedef void (*Device_ICM45686_DelayMs)(
    void *context,
    uint32_t milliseconds);

typedef struct {
    Device_ICM45686_ReadRegisters read_registers;
    Device_ICM45686_WriteRegister write_register;
    Device_ICM45686_DelayMs delay_ms;
    void *transport_context;
    Device_ICM45686_AccelRange accelerometer_range;
    Device_ICM45686_GyroRange gyroscope_range;
    Device_ICM45686_Odr accelerometer_odr;
    Device_ICM45686_Odr gyroscope_odr;
    Device_ICM45686_DataEndianness data_endianness;
    bool enable_data_ready_interrupt;
} Device_ICM45686_Config;

/* Init currently accepts LITTLE_ENDIAN only because soft reset restores that
 * setting and this Device intentionally does not write indirect registers. */

typedef enum {
    DEVICE_ICM45686_STATUS_OK = 0,
    DEVICE_ICM45686_STATUS_INVALID_ARGUMENT,
    DEVICE_ICM45686_STATUS_TRANSPORT_ERROR,
    DEVICE_ICM45686_STATUS_ID_MISMATCH,
    DEVICE_ICM45686_STATUS_CONFIGURATION_MISMATCH,
    DEVICE_ICM45686_STATUS_NOT_INITIALIZED,
    DEVICE_ICM45686_STATUS_NONFINITE_RESULT,
    DEVICE_ICM45686_STATUS_IMU_REJECTED_SAMPLE,
} Device_ICM45686_Status;

typedef struct {
    float acceleration_mps2[3];
    float angular_rate_dps[3];
    float temperature_celsius;
    uint32_t sampled_at_ms;
    float dt_seconds;
    uint32_t sequence;
    bool valid;
} Device_ICM45686_Sample;

typedef struct {
    Device_ICM45686_Config config;
    float acceleration_scale_mps2_per_count;
    float angular_rate_scale_dps_per_count;
    uint32_t sample_count;
    Device_ICM45686_Status last_status;
    bool initialized;
} Device_ICM45686;

Device_ICM45686_Config Device_ICM45686_DefaultConfig(void);
Device_ICM45686_Status Device_ICM45686_Init(
    Device_ICM45686 *device,
    const Device_ICM45686_Config *config);
Device_ICM45686_Status Device_ICM45686_ReadSample(
    Device_ICM45686 *device,
    uint32_t sampled_at_ms,
    float dt_seconds,
    Device_ICM45686_Sample *sample);

/* Decode the 14-byte sensor-data burst beginning at register 0x00. The
 * selected byte order must match SREG_DATA_ENDIAN_SEL. Reset default is
 * little-endian; this driver does not modify the indirect endian register. */
bool Device_ICM45686_DecodeSensorBurst(
    Device_ICM45686_AccelRange accelerometer_range,
    Device_ICM45686_GyroRange gyroscope_range,
    Device_ICM45686_DataEndianness endianness,
    const uint8_t bytes[14],
    float acceleration_mps2[3],
    float angular_rate_dps[3],
    float *temperature_celsius);

/* Read, convert, and push one physical-unit sample through the common IMU
 * axis mapping, calibration and Attitude algorithm. Device_IMU must be
 * configured with both input scales equal to 1.0. */
Device_ICM45686_Status Device_ICM45686_UpdateIMU(
    Device_ICM45686 *device,
    Device_IMU *imu,
    uint32_t sampled_at_ms,
    float dt_seconds,
    Device_ICM45686_Sample *sample);

#endif
