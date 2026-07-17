#ifndef DEVICE_SENSOR_BMI088_DEVICE_BMI088_H
#define DEVICE_SENSOR_BMI088_DEVICE_BMI088_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    DEVICE_BMI088_DIE_ACCELEROMETER = 0,
    DEVICE_BMI088_DIE_GYROSCOPE,
} Device_BMI088_Die;

typedef enum {
    DEVICE_BMI088_ACCEL_RANGE_3G = 0,
    DEVICE_BMI088_ACCEL_RANGE_6G,
    DEVICE_BMI088_ACCEL_RANGE_12G,
    DEVICE_BMI088_ACCEL_RANGE_24G,
} Device_BMI088_AccelRange;

typedef enum {
    DEVICE_BMI088_GYRO_RANGE_2000_DPS = 0,
    DEVICE_BMI088_GYRO_RANGE_1000_DPS,
    DEVICE_BMI088_GYRO_RANGE_500_DPS,
    DEVICE_BMI088_GYRO_RANGE_250_DPS,
    DEVICE_BMI088_GYRO_RANGE_125_DPS,
} Device_BMI088_GyroRange;

typedef enum {
    DEVICE_BMI088_ACCEL_ODR_100_HZ = 0x08,
    DEVICE_BMI088_ACCEL_ODR_200_HZ = 0x09,
    DEVICE_BMI088_ACCEL_ODR_400_HZ = 0x0A,
    DEVICE_BMI088_ACCEL_ODR_800_HZ = 0x0B,
    DEVICE_BMI088_ACCEL_ODR_1600_HZ = 0x0C,
} Device_BMI088_AccelOdr;

typedef enum {
    DEVICE_BMI088_GYRO_BW_2000_532_HZ = 0x00,
    DEVICE_BMI088_GYRO_BW_2000_230_HZ = 0x01,
    DEVICE_BMI088_GYRO_BW_1000_116_HZ = 0x02,
    DEVICE_BMI088_GYRO_BW_400_47_HZ = 0x03,
    DEVICE_BMI088_GYRO_BW_200_23_HZ = 0x04,
    DEVICE_BMI088_GYRO_BW_100_12_HZ = 0x05,
    DEVICE_BMI088_GYRO_BW_200_64_HZ = 0x06,
    DEVICE_BMI088_GYRO_BW_100_32_HZ = 0x07,
} Device_BMI088_GyroBandwidth;

typedef bool (*Device_BMI088_ReadRegisters)(
    void *context,
    Device_BMI088_Die die,
    uint8_t first_register,
    uint8_t *bytes,
    size_t length);
typedef bool (*Device_BMI088_WriteRegister)(
    void *context,
    Device_BMI088_Die die,
    uint8_t register_address,
    uint8_t value);
typedef void (*Device_BMI088_DelayMs)(void *context, uint32_t milliseconds);

typedef struct {
    Device_BMI088_ReadRegisters read_registers;
    Device_BMI088_WriteRegister write_register;
    Device_BMI088_DelayMs delay_ms;
    void *transport_context;
    Device_BMI088_AccelRange accelerometer_range;
    Device_BMI088_GyroRange gyroscope_range;
    Device_BMI088_AccelOdr accelerometer_odr;
    Device_BMI088_GyroBandwidth gyroscope_bandwidth;
} Device_BMI088_Config;

typedef enum {
    DEVICE_BMI088_STATUS_OK = 0,
    DEVICE_BMI088_STATUS_INVALID_ARGUMENT,
    DEVICE_BMI088_STATUS_TRANSPORT_ERROR,
    DEVICE_BMI088_STATUS_ACCELEROMETER_ID_MISMATCH,
    DEVICE_BMI088_STATUS_GYROSCOPE_ID_MISMATCH,
    DEVICE_BMI088_STATUS_NOT_INITIALIZED,
    DEVICE_BMI088_STATUS_NONFINITE_RESULT,
} Device_BMI088_Status;

typedef struct {
    float acceleration_mps2[3];
    float angular_rate_dps[3];
    float temperature_celsius;
    uint32_t sampled_at_ms;
    float dt_seconds;
    uint32_t sequence;
    bool valid;
} Device_BMI088_Sample;

typedef struct {
    Device_BMI088_Config config;
    float acceleration_scale_mps2_per_count;
    float angular_rate_scale_dps_per_count;
    uint32_t sample_count;
    Device_BMI088_Status last_status;
    bool initialized;
} Device_BMI088;

Device_BMI088_Config Device_BMI088_DefaultConfig(void);
Device_BMI088_Status Device_BMI088_Init(
    Device_BMI088 *device, const Device_BMI088_Config *config);

/* The backend owns SPI chip-select, dummy-byte and DMA details. Reads here
 * start at the documented data register and return only payload bytes. */
Device_BMI088_Status Device_BMI088_ReadSample(
    Device_BMI088 *device,
    uint32_t sampled_at_ms,
    float dt_seconds,
    Device_BMI088_Sample *sample);

/* Public decoders make DMA completion paths testable without performing bus
 * I/O. Failure leaves the output array/value unchanged. */
bool Device_BMI088_DecodeAcceleration(
    Device_BMI088_AccelRange range,
    const uint8_t bytes[6],
    float acceleration_mps2[3]);
bool Device_BMI088_DecodeAngularRate(
    Device_BMI088_GyroRange range,
    const uint8_t bytes[6],
    float angular_rate_dps[3]);
bool Device_BMI088_DecodeTemperature(
    const uint8_t bytes[2], float *temperature_celsius);

#endif
