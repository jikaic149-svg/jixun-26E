#include "device_icm45686.h"

#include <math.h>
#include <stddef.h>

#define DEVICE_ICM45686_GRAVITY_MPS2 9.80665F

#define DEVICE_ICM45686_REG_ACCEL_DATA UINT8_C(0x00)
#define DEVICE_ICM45686_REG_PWR_MGMT0 UINT8_C(0x10)
#define DEVICE_ICM45686_REG_INT1_CONFIG0 UINT8_C(0x16)
#define DEVICE_ICM45686_REG_INT1_CONFIG2 UINT8_C(0x18)
#define DEVICE_ICM45686_REG_ACCEL_CONFIG0 UINT8_C(0x1B)
#define DEVICE_ICM45686_REG_GYRO_CONFIG0 UINT8_C(0x1C)
#define DEVICE_ICM45686_REG_WHO_AM_I UINT8_C(0x72)
#define DEVICE_ICM45686_REG_MISC2 UINT8_C(0x7F)

#define DEVICE_ICM45686_SOFT_RESET UINT8_C(0x02)
#define DEVICE_ICM45686_INT1_DATA_READY UINT8_C(0x04)
#define DEVICE_ICM45686_INT1_PUSH_PULL_ACTIVE_HIGH UINT8_C(0x01)
#define DEVICE_ICM45686_ACCEL_GYRO_LOW_NOISE UINT8_C(0x0F)

static bool valid_accel_range(Device_ICM45686_AccelRange range)
{
    return (unsigned int)range <=
        (unsigned int)DEVICE_ICM45686_ACCEL_RANGE_2G;
}

static bool valid_gyro_range(Device_ICM45686_GyroRange range)
{
    return (unsigned int)range <=
        (unsigned int)DEVICE_ICM45686_GYRO_RANGE_15_625_DPS;
}

static bool valid_odr(Device_ICM45686_Odr odr)
{
    return ((unsigned int)odr >=
            (unsigned int)DEVICE_ICM45686_ODR_6400_HZ) &&
        ((unsigned int)odr <=
            (unsigned int)DEVICE_ICM45686_ODR_1_5625_HZ);
}

static bool valid_endianness(Device_ICM45686_DataEndianness endianness)
{
    return (endianness == DEVICE_ICM45686_DATA_LITTLE_ENDIAN) ||
        (endianness == DEVICE_ICM45686_DATA_BIG_ENDIAN);
}

static float accel_full_scale_g(Device_ICM45686_AccelRange range)
{
    static const float values[] = {32.0F, 16.0F, 8.0F, 4.0F, 2.0F};
    return values[(unsigned int)range];
}

static float gyro_full_scale_dps(Device_ICM45686_GyroRange range)
{
    static const float values[] = {
        4000.0F,
        2000.0F,
        1000.0F,
        500.0F,
        250.0F,
        125.0F,
        62.5F,
        31.25F,
        15.625F,
    };
    return values[(unsigned int)range];
}

static int16_t decode_i16(
    const uint8_t bytes[2], Device_ICM45686_DataEndianness endianness)
{
    uint16_t value;

    if (endianness == DEVICE_ICM45686_DATA_LITTLE_ENDIAN) {
        value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
    } else {
        value = ((uint16_t)bytes[0] << 8U) | (uint16_t)bytes[1];
    }
    if (value <= UINT16_C(0x7FFF)) {
        return (int16_t)value;
    }
    return (int16_t)((int32_t)value - INT32_C(65536));
}

static bool read_register(
    const Device_ICM45686_Config *config,
    uint8_t address,
    uint8_t *value)
{
    return config->read_registers(
        config->transport_context, address, value, 1U);
}

static bool write_register(
    const Device_ICM45686_Config *config,
    uint8_t address,
    uint8_t value)
{
    return config->write_register(
        config->transport_context, address, value);
}

static bool read_matches(
    const Device_ICM45686_Config *config,
    uint8_t address,
    uint8_t expected)
{
    uint8_t actual;

    return read_register(config, address, &actual) && (actual == expected);
}

static uint8_t accel_config_value(const Device_ICM45686_Config *config)
{
    return (uint8_t)(((uint8_t)config->accelerometer_range << 4U) |
        (uint8_t)config->accelerometer_odr);
}

static uint8_t gyro_config_value(const Device_ICM45686_Config *config)
{
    return (uint8_t)(((uint8_t)config->gyroscope_range << 4U) |
        (uint8_t)config->gyroscope_odr);
}

Device_ICM45686_Config Device_ICM45686_DefaultConfig(void)
{
    return (Device_ICM45686_Config){
        .read_registers = NULL,
        .write_register = NULL,
        .delay_ms = NULL,
        .transport_context = NULL,
        .accelerometer_range = DEVICE_ICM45686_ACCEL_RANGE_4G,
        .gyroscope_range = DEVICE_ICM45686_GYRO_RANGE_1000_DPS,
        .accelerometer_odr = DEVICE_ICM45686_ODR_800_HZ,
        .gyroscope_odr = DEVICE_ICM45686_ODR_800_HZ,
        .data_endianness = DEVICE_ICM45686_DATA_LITTLE_ENDIAN,
        .enable_data_ready_interrupt = true,
    };
}

Device_ICM45686_Status Device_ICM45686_Init(
    Device_ICM45686 *device,
    const Device_ICM45686_Config *config)
{
    Device_ICM45686 next = {0};
    uint8_t identifier;
    uint8_t interrupt_enable;

    if (device != NULL) {
        *device = (Device_ICM45686){0};
    }
    if ((device == NULL) || (config == NULL) ||
        (config->read_registers == NULL) ||
        (config->write_register == NULL) ||
        (config->delay_ms == NULL) ||
        !valid_accel_range(config->accelerometer_range) ||
        !valid_gyro_range(config->gyroscope_range) ||
        !valid_odr(config->accelerometer_odr) ||
        !valid_odr(config->gyroscope_odr) ||
        !valid_endianness(config->data_endianness) ||
        (config->data_endianness !=
            DEVICE_ICM45686_DATA_LITTLE_ENDIAN)) {
        /* Soft reset restores little-endian sensor data.  Init must not accept
         * a big-endian decode request until the indirect endian register is
         * explicitly implemented and verified.  The standalone decoder keeps
         * big-endian support for test/DMA buffers configured elsewhere. */
        return DEVICE_ICM45686_STATUS_INVALID_ARGUMENT;
    }

    config->delay_ms(config->transport_context, 2U);
    if (!read_register(config, DEVICE_ICM45686_REG_WHO_AM_I, &identifier)) {
        return DEVICE_ICM45686_STATUS_TRANSPORT_ERROR;
    }
    if (identifier != DEVICE_ICM45686_WHO_AM_I_VALUE) {
        return DEVICE_ICM45686_STATUS_ID_MISMATCH;
    }
    if (!write_register(
            config, DEVICE_ICM45686_REG_MISC2,
            DEVICE_ICM45686_SOFT_RESET)) {
        return DEVICE_ICM45686_STATUS_TRANSPORT_ERROR;
    }
    config->delay_ms(config->transport_context, 2U);
    if (!read_matches(
            config,
            DEVICE_ICM45686_REG_WHO_AM_I,
            DEVICE_ICM45686_WHO_AM_I_VALUE)) {
        return DEVICE_ICM45686_STATUS_ID_MISMATCH;
    }

    interrupt_enable = config->enable_data_ready_interrupt ?
        DEVICE_ICM45686_INT1_DATA_READY : UINT8_C(0x00);
    if (!write_register(
            config,
            DEVICE_ICM45686_REG_ACCEL_CONFIG0,
            accel_config_value(config)) ||
        !write_register(
            config,
            DEVICE_ICM45686_REG_GYRO_CONFIG0,
            gyro_config_value(config)) ||
        !write_register(
            config,
            DEVICE_ICM45686_REG_INT1_CONFIG2,
            DEVICE_ICM45686_INT1_PUSH_PULL_ACTIVE_HIGH) ||
        !write_register(
            config,
            DEVICE_ICM45686_REG_INT1_CONFIG0,
            interrupt_enable) ||
        !write_register(
            config,
            DEVICE_ICM45686_REG_PWR_MGMT0,
            DEVICE_ICM45686_ACCEL_GYRO_LOW_NOISE)) {
        return DEVICE_ICM45686_STATUS_TRANSPORT_ERROR;
    }
    config->delay_ms(config->transport_context, 35U);
    if (!read_matches(
            config,
            DEVICE_ICM45686_REG_ACCEL_CONFIG0,
            accel_config_value(config)) ||
        !read_matches(
            config,
            DEVICE_ICM45686_REG_GYRO_CONFIG0,
            gyro_config_value(config)) ||
        !read_matches(
            config,
            DEVICE_ICM45686_REG_PWR_MGMT0,
            DEVICE_ICM45686_ACCEL_GYRO_LOW_NOISE)) {
        return DEVICE_ICM45686_STATUS_CONFIGURATION_MISMATCH;
    }

    next.config = *config;
    next.acceleration_scale_mps2_per_count =
        accel_full_scale_g(config->accelerometer_range) *
        DEVICE_ICM45686_GRAVITY_MPS2 / 32768.0F;
    next.angular_rate_scale_dps_per_count =
        gyro_full_scale_dps(config->gyroscope_range) / 32768.0F;
    next.last_status = DEVICE_ICM45686_STATUS_OK;
    next.initialized = true;
    *device = next;
    return DEVICE_ICM45686_STATUS_OK;
}

bool Device_ICM45686_DecodeSensorBurst(
    Device_ICM45686_AccelRange accelerometer_range,
    Device_ICM45686_GyroRange gyroscope_range,
    Device_ICM45686_DataEndianness endianness,
    const uint8_t bytes[14],
    float acceleration_mps2[3],
    float angular_rate_dps[3],
    float *temperature_celsius)
{
    float next_acceleration[3];
    float next_angular_rate[3];
    float next_temperature;
    float acceleration_scale;
    float angular_rate_scale;
    unsigned int axis;

    if (!valid_accel_range(accelerometer_range) ||
        !valid_gyro_range(gyroscope_range) ||
        !valid_endianness(endianness) || (bytes == NULL) ||
        (acceleration_mps2 == NULL) || (angular_rate_dps == NULL) ||
        (temperature_celsius == NULL)) {
        return false;
    }
    acceleration_scale = accel_full_scale_g(accelerometer_range) *
        DEVICE_ICM45686_GRAVITY_MPS2 / 32768.0F;
    angular_rate_scale = gyro_full_scale_dps(gyroscope_range) / 32768.0F;
    for (axis = 0U; axis < 3U; ++axis) {
        next_acceleration[axis] =
            (float)decode_i16(&bytes[2U * axis], endianness) *
            acceleration_scale;
        next_angular_rate[axis] =
            (float)decode_i16(&bytes[6U + 2U * axis], endianness) *
            angular_rate_scale;
        if (!isfinite(next_acceleration[axis]) ||
            !isfinite(next_angular_rate[axis])) {
            return false;
        }
    }
    next_temperature =
        (float)decode_i16(&bytes[12], endianness) / 128.0F + 25.0F;
    if (!isfinite(next_temperature)) {
        return false;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        acceleration_mps2[axis] = next_acceleration[axis];
        angular_rate_dps[axis] = next_angular_rate[axis];
    }
    *temperature_celsius = next_temperature;
    return true;
}

Device_ICM45686_Status Device_ICM45686_ReadSample(
    Device_ICM45686 *device,
    uint32_t sampled_at_ms,
    float dt_seconds,
    Device_ICM45686_Sample *sample)
{
    uint8_t bytes[14];
    Device_ICM45686_Sample next = {0};

    if ((device == NULL) || (sample == NULL)) {
        return DEVICE_ICM45686_STATUS_INVALID_ARGUMENT;
    }
    if (!device->initialized) {
        return DEVICE_ICM45686_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(dt_seconds) || (dt_seconds <= 0.0F)) {
        device->last_status = DEVICE_ICM45686_STATUS_INVALID_ARGUMENT;
        return device->last_status;
    }
    if (!device->config.read_registers(
            device->config.transport_context,
            DEVICE_ICM45686_REG_ACCEL_DATA,
            bytes,
            sizeof(bytes))) {
        device->last_status = DEVICE_ICM45686_STATUS_TRANSPORT_ERROR;
        return device->last_status;
    }
    if (!Device_ICM45686_DecodeSensorBurst(
            device->config.accelerometer_range,
            device->config.gyroscope_range,
            device->config.data_endianness,
            bytes,
            next.acceleration_mps2,
            next.angular_rate_dps,
            &next.temperature_celsius)) {
        device->last_status = DEVICE_ICM45686_STATUS_NONFINITE_RESULT;
        return device->last_status;
    }
    next.sampled_at_ms = sampled_at_ms;
    next.dt_seconds = dt_seconds;
    next.sequence = (device->sample_count == UINT32_MAX) ?
        UINT32_MAX : device->sample_count + 1U;
    next.valid = true;
    device->sample_count = next.sequence;
    device->last_status = DEVICE_ICM45686_STATUS_OK;
    *sample = next;
    return DEVICE_ICM45686_STATUS_OK;
}

Device_ICM45686_Status Device_ICM45686_UpdateIMU(
    Device_ICM45686 *device,
    Device_IMU *imu,
    uint32_t sampled_at_ms,
    float dt_seconds,
    Device_ICM45686_Sample *sample)
{
    Device_ICM45686_Sample next;
    Device_IMU_Sample imu_sample;
    Device_ICM45686_Status status;
    unsigned int axis;

    if ((device == NULL) || (imu == NULL)) {
        return DEVICE_ICM45686_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(imu->config.acceleration_mps2_per_sensor_unit) ||
        !isfinite(imu->config.angular_rate_dps_per_sensor_unit) ||
        (imu->config.acceleration_mps2_per_sensor_unit != 1.0F) ||
        (imu->config.angular_rate_dps_per_sensor_unit != 1.0F)) {
        device->last_status = DEVICE_ICM45686_STATUS_INVALID_ARGUMENT;
        return device->last_status;
    }
    status = Device_ICM45686_ReadSample(
        device, sampled_at_ms, dt_seconds, &next);
    if (status != DEVICE_ICM45686_STATUS_OK) {
        return status;
    }
    imu_sample = (Device_IMU_Sample){
        .sampled_at_ms = sampled_at_ms,
        .dt_seconds = dt_seconds,
    };
    for (axis = 0U; axis < 3U; ++axis) {
        imu_sample.acceleration_sensor_units[axis] =
            next.acceleration_mps2[axis];
        imu_sample.angular_rate_sensor_units[axis] =
            next.angular_rate_dps[axis];
    }
    if (!Device_IMU_PushSample(imu, &imu_sample)) {
        device->last_status = DEVICE_ICM45686_STATUS_IMU_REJECTED_SAMPLE;
        return device->last_status;
    }
    if (sample != NULL) {
        *sample = next;
    }
    device->last_status = DEVICE_ICM45686_STATUS_OK;
    return device->last_status;
}
