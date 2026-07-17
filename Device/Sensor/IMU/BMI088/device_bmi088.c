#include "device_bmi088.h"

#include <math.h>
#include <stddef.h>

#define DEVICE_BMI088_GRAVITY_MPS2 9.80665F

#define DEVICE_BMI088_ACCEL_CHIP_ID_REG 0x00U
#define DEVICE_BMI088_ACCEL_CHIP_ID_VALUE 0x1EU
#define DEVICE_BMI088_ACCEL_DATA_REG 0x12U
#define DEVICE_BMI088_ACCEL_TEMP_REG 0x22U
#define DEVICE_BMI088_ACCEL_CONF_REG 0x40U
#define DEVICE_BMI088_ACCEL_RANGE_REG 0x41U
#define DEVICE_BMI088_ACCEL_PWR_CONF_REG 0x7CU
#define DEVICE_BMI088_ACCEL_PWR_CTRL_REG 0x7DU
#define DEVICE_BMI088_ACCEL_SOFTRESET_REG 0x7EU

#define DEVICE_BMI088_GYRO_CHIP_ID_REG 0x00U
#define DEVICE_BMI088_GYRO_CHIP_ID_VALUE 0x0FU
#define DEVICE_BMI088_GYRO_DATA_REG 0x02U
#define DEVICE_BMI088_GYRO_RANGE_REG 0x0FU
#define DEVICE_BMI088_GYRO_BANDWIDTH_REG 0x10U
#define DEVICE_BMI088_GYRO_POWER_REG 0x11U
#define DEVICE_BMI088_GYRO_SOFTRESET_REG 0x14U

#define DEVICE_BMI088_SOFTRESET_VALUE 0xB6U

static bool valid_accel_range(Device_BMI088_AccelRange range)
{
    return (unsigned int)range <=
        (unsigned int)DEVICE_BMI088_ACCEL_RANGE_24G;
}

static bool valid_gyro_range(Device_BMI088_GyroRange range)
{
    return (unsigned int)range <=
        (unsigned int)DEVICE_BMI088_GYRO_RANGE_125_DPS;
}

static bool valid_accel_odr(Device_BMI088_AccelOdr odr)
{
    return ((unsigned int)odr >=
            (unsigned int)DEVICE_BMI088_ACCEL_ODR_100_HZ) &&
        ((unsigned int)odr <=
            (unsigned int)DEVICE_BMI088_ACCEL_ODR_1600_HZ);
}

static bool valid_gyro_bandwidth(Device_BMI088_GyroBandwidth bandwidth)
{
    return (unsigned int)bandwidth <=
        (unsigned int)DEVICE_BMI088_GYRO_BW_100_32_HZ;
}

static float accel_full_scale_g(Device_BMI088_AccelRange range)
{
    static const float values[] = {3.0F, 6.0F, 12.0F, 24.0F};
    return values[(unsigned int)range];
}

static float gyro_full_scale_dps(Device_BMI088_GyroRange range)
{
    static const float values[] = {
        2000.0F, 1000.0F, 500.0F, 250.0F, 125.0F};
    return values[(unsigned int)range];
}

static int16_t decode_i16_le(const uint8_t bytes[2])
{
    const uint16_t value = (uint16_t)bytes[0] |
        ((uint16_t)bytes[1] << 8U);
    if (value <= UINT16_C(0x7FFF)) {
        return (int16_t)value;
    }
    return (int16_t)((int32_t)value - INT32_C(65536));
}

static bool decode_vector(
    const uint8_t bytes[6], float scale, float output[3])
{
    float next[3];
    unsigned int axis;

    if ((bytes == NULL) || (output == NULL) || !isfinite(scale) ||
        (scale <= 0.0F)) {
        return false;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        next[axis] = (float)decode_i16_le(&bytes[2U * axis]) * scale;
        if (!isfinite(next[axis])) {
            return false;
        }
    }
    output[0] = next[0];
    output[1] = next[1];
    output[2] = next[2];
    return true;
}

static bool write_checked(
    const Device_BMI088_Config *config,
    Device_BMI088_Die die,
    uint8_t address,
    uint8_t value)
{
    return config->write_register(
        config->transport_context, die, address, value);
}

static bool read_id(
    const Device_BMI088_Config *config,
    Device_BMI088_Die die,
    uint8_t *identifier)
{
    return config->read_registers(
        config->transport_context, die, 0x00U, identifier, 1U);
}

Device_BMI088_Config Device_BMI088_DefaultConfig(void)
{
    const Device_BMI088_Config config = {
        .read_registers = NULL,
        .write_register = NULL,
        .delay_ms = NULL,
        .transport_context = NULL,
        .accelerometer_range = DEVICE_BMI088_ACCEL_RANGE_3G,
        .gyroscope_range = DEVICE_BMI088_GYRO_RANGE_2000_DPS,
        .accelerometer_odr = DEVICE_BMI088_ACCEL_ODR_400_HZ,
        .gyroscope_bandwidth = DEVICE_BMI088_GYRO_BW_1000_116_HZ,
    };
    return config;
}

Device_BMI088_Status Device_BMI088_Init(
    Device_BMI088 *device, const Device_BMI088_Config *config)
{
    Device_BMI088 next = {0};
    uint8_t identifier;

    if (device != NULL) {
        *device = (Device_BMI088){0};
    }
    if ((device == NULL) || (config == NULL) ||
        (config->read_registers == NULL) ||
        (config->write_register == NULL) ||
        (config->delay_ms == NULL) ||
        !valid_accel_range(config->accelerometer_range) ||
        !valid_gyro_range(config->gyroscope_range) ||
        !valid_accel_odr(config->accelerometer_odr) ||
        !valid_gyro_bandwidth(config->gyroscope_bandwidth)) {
        return DEVICE_BMI088_STATUS_INVALID_ARGUMENT;
    }

    if (!read_id(config, DEVICE_BMI088_DIE_ACCELEROMETER, &identifier)) {
        return DEVICE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    if (identifier != DEVICE_BMI088_ACCEL_CHIP_ID_VALUE) {
        return DEVICE_BMI088_STATUS_ACCELEROMETER_ID_MISMATCH;
    }
    if (!read_id(config, DEVICE_BMI088_DIE_GYROSCOPE, &identifier)) {
        return DEVICE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    if (identifier != DEVICE_BMI088_GYRO_CHIP_ID_VALUE) {
        return DEVICE_BMI088_STATUS_GYROSCOPE_ID_MISMATCH;
    }

    if (!write_checked(config, DEVICE_BMI088_DIE_ACCELEROMETER,
            DEVICE_BMI088_ACCEL_SOFTRESET_REG,
            DEVICE_BMI088_SOFTRESET_VALUE)) {
        return DEVICE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    config->delay_ms(config->transport_context, 50U);
    if (!write_checked(config, DEVICE_BMI088_DIE_ACCELEROMETER,
            DEVICE_BMI088_ACCEL_PWR_CTRL_REG, 0x04U) ||
        !write_checked(config, DEVICE_BMI088_DIE_ACCELEROMETER,
            DEVICE_BMI088_ACCEL_PWR_CONF_REG, 0x00U) ||
        !write_checked(config, DEVICE_BMI088_DIE_ACCELEROMETER,
            DEVICE_BMI088_ACCEL_CONF_REG,
            (uint8_t)(0xA0U | (uint8_t)config->accelerometer_odr)) ||
        !write_checked(config, DEVICE_BMI088_DIE_ACCELEROMETER,
            DEVICE_BMI088_ACCEL_RANGE_REG,
            (uint8_t)config->accelerometer_range)) {
        return DEVICE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    config->delay_ms(config->transport_context, 10U);

    if (!write_checked(config, DEVICE_BMI088_DIE_GYROSCOPE,
            DEVICE_BMI088_GYRO_SOFTRESET_REG,
            DEVICE_BMI088_SOFTRESET_VALUE)) {
        return DEVICE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    config->delay_ms(config->transport_context, 50U);
    if (!write_checked(config, DEVICE_BMI088_DIE_GYROSCOPE,
            DEVICE_BMI088_GYRO_POWER_REG, 0x00U) ||
        !write_checked(config, DEVICE_BMI088_DIE_GYROSCOPE,
            DEVICE_BMI088_GYRO_RANGE_REG,
            (uint8_t)config->gyroscope_range) ||
        !write_checked(config, DEVICE_BMI088_DIE_GYROSCOPE,
            DEVICE_BMI088_GYRO_BANDWIDTH_REG,
            (uint8_t)config->gyroscope_bandwidth)) {
        return DEVICE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    config->delay_ms(config->transport_context, 30U);

    if (!read_id(config, DEVICE_BMI088_DIE_ACCELEROMETER, &identifier)) {
        return DEVICE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    if (identifier != DEVICE_BMI088_ACCEL_CHIP_ID_VALUE) {
        return DEVICE_BMI088_STATUS_ACCELEROMETER_ID_MISMATCH;
    }
    if (!read_id(config, DEVICE_BMI088_DIE_GYROSCOPE, &identifier)) {
        return DEVICE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    if (identifier != DEVICE_BMI088_GYRO_CHIP_ID_VALUE) {
        return DEVICE_BMI088_STATUS_GYROSCOPE_ID_MISMATCH;
    }

    next.config = *config;
    next.acceleration_scale_mps2_per_count =
        accel_full_scale_g(config->accelerometer_range) *
        DEVICE_BMI088_GRAVITY_MPS2 / 32768.0F;
    next.angular_rate_scale_dps_per_count =
        gyro_full_scale_dps(config->gyroscope_range) / 32768.0F;
    next.last_status = DEVICE_BMI088_STATUS_OK;
    next.initialized = true;
    *device = next;
    return DEVICE_BMI088_STATUS_OK;
}

bool Device_BMI088_DecodeAcceleration(
    Device_BMI088_AccelRange range,
    const uint8_t bytes[6],
    float acceleration_mps2[3])
{
    if (!valid_accel_range(range)) {
        return false;
    }
    return decode_vector(bytes,
        accel_full_scale_g(range) * DEVICE_BMI088_GRAVITY_MPS2 / 32768.0F,
        acceleration_mps2);
}

bool Device_BMI088_DecodeAngularRate(
    Device_BMI088_GyroRange range,
    const uint8_t bytes[6],
    float angular_rate_dps[3])
{
    if (!valid_gyro_range(range)) {
        return false;
    }
    return decode_vector(bytes,
        gyro_full_scale_dps(range) / 32768.0F, angular_rate_dps);
}

bool Device_BMI088_DecodeTemperature(
    const uint8_t bytes[2], float *temperature_celsius)
{
    int32_t raw;
    float next;

    if ((bytes == NULL) || (temperature_celsius == NULL)) {
        return false;
    }
    raw = ((int32_t)bytes[0] << 3) | ((int32_t)bytes[1] >> 5);
    if (raw > 1023) {
        raw -= 2048;
    }
    next = ((float)raw * 0.125F) + 23.0F;
    if (!isfinite(next)) {
        return false;
    }
    *temperature_celsius = next;
    return true;
}

Device_BMI088_Status Device_BMI088_ReadSample(
    Device_BMI088 *device,
    uint32_t sampled_at_ms,
    float dt_seconds,
    Device_BMI088_Sample *sample)
{
    uint8_t accel_bytes[6];
    uint8_t gyro_bytes[6];
    uint8_t temperature_bytes[2];
    Device_BMI088_Sample next = {0};

    if ((device == NULL) || (sample == NULL)) {
        return DEVICE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    if (!device->initialized) {
        return DEVICE_BMI088_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(dt_seconds) || (dt_seconds <= 0.0F)) {
        device->last_status = DEVICE_BMI088_STATUS_INVALID_ARGUMENT;
        return device->last_status;
    }
    if (!device->config.read_registers(device->config.transport_context,
            DEVICE_BMI088_DIE_ACCELEROMETER,
            DEVICE_BMI088_ACCEL_DATA_REG, accel_bytes, sizeof(accel_bytes)) ||
        !device->config.read_registers(device->config.transport_context,
            DEVICE_BMI088_DIE_GYROSCOPE,
            DEVICE_BMI088_GYRO_DATA_REG, gyro_bytes, sizeof(gyro_bytes)) ||
        !device->config.read_registers(device->config.transport_context,
            DEVICE_BMI088_DIE_ACCELEROMETER,
            DEVICE_BMI088_ACCEL_TEMP_REG,
            temperature_bytes, sizeof(temperature_bytes))) {
        device->last_status = DEVICE_BMI088_STATUS_TRANSPORT_ERROR;
        return device->last_status;
    }
    if (!Device_BMI088_DecodeAcceleration(
            device->config.accelerometer_range,
            accel_bytes, next.acceleration_mps2) ||
        !Device_BMI088_DecodeAngularRate(
            device->config.gyroscope_range,
            gyro_bytes, next.angular_rate_dps) ||
        !Device_BMI088_DecodeTemperature(
            temperature_bytes, &next.temperature_celsius)) {
        device->last_status = DEVICE_BMI088_STATUS_NONFINITE_RESULT;
        return device->last_status;
    }
    next.sampled_at_ms = sampled_at_ms;
    next.dt_seconds = dt_seconds;
    next.sequence = (device->sample_count == UINT32_MAX) ? UINT32_MAX :
        device->sample_count + 1U;
    next.valid = true;
    device->sample_count = next.sequence;
    device->last_status = DEVICE_BMI088_STATUS_OK;
    *sample = next;
    return DEVICE_BMI088_STATUS_OK;
}
