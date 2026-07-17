#include "mode_gimbal_internal.h"

#include <stddef.h>

static bool valid_scheduler_mode(Mode_Gimbal_SchedulerMode mode)
{
    return mode <= MODE_GIMBAL_SCHEDULER_GRAVITY_CALIB;
}

static void scheduler_stop_output(Mode_Gimbal_SchedulerOutput *output)
{
    if (output == NULL) {
        return;
    }
    Mode_Gimbal_Internal_StopIntent(&output->current);
    output->active_mode = MODE_GIMBAL_SCHEDULER_IDLE;
    output->mode_entered = false;
    output->encoder_lock_status =
        MODE_GIMBAL_STAB_CALIB_WAITING_FEEDBACK;
    output->vision_active = false;
    output->vision_scanning = false;
    output->gravity_status = MODE_GIMBAL_GRAVITY_WAITING_FEEDBACK;
    output->gravity_sample_ready = false;
    output->gravity_sample.encoder_angle_deg = 0.0F;
    output->gravity_sample.measured_current = 0.0F;
}

Mode_Gimbal_SchedulerConfig Mode_Gimbal_Scheduler_DefaultConfig(void)
{
    Mode_Gimbal_SchedulerConfig config;

    config.encoder_lock = Mode_Gimbal_StabCalib_DefaultConfig();
    config.gimbal_stab = Mode_Gimbal_GimbalStab_DefaultConfig();
    config.vision_stab = Mode_Gimbal_VisionStab_DefaultConfig();
    config.gravity_calib = Mode_Gimbal_GravityCalib_DefaultConfig();
    return config;
}

bool Mode_Gimbal_Scheduler_Init(
    Mode_Gimbal_SchedulerContext *context,
    const Mode_Gimbal_SchedulerConfig *config)
{
    Mode_Gimbal_StabCalibContext encoder_lock;
    Mode_Gimbal_GimbalStabContext gimbal_stab;
    Mode_Gimbal_VisionStabContext vision_stab;
    Mode_Gimbal_GravityCalibContext gravity_calib;

    if ((context == NULL) || (config == NULL) ||
        !Mode_Gimbal_StabCalib_Init(
            &encoder_lock, &config->encoder_lock) ||
        !Mode_Gimbal_GimbalStab_Init(
            &gimbal_stab, &config->gimbal_stab) ||
        !Mode_Gimbal_VisionStab_Init(
            &vision_stab, &config->vision_stab) ||
        !Mode_Gimbal_GravityCalib_Init(
            &gravity_calib, &config->gravity_calib)) {
        return false;
    }

    context->config = *config;
    context->encoder_lock = encoder_lock;
    context->gimbal_stab = gimbal_stab;
    context->vision_stab = vision_stab;
    context->gravity_calib = gravity_calib;
    context->active_mode = MODE_GIMBAL_SCHEDULER_IDLE;
    context->active_mode_valid = false;
    context->initialized = true;
    return true;
}

static bool enter_mode(
    Mode_Gimbal_SchedulerContext *context,
    Mode_Gimbal_SchedulerMode mode)
{
    switch (mode) {
        case MODE_GIMBAL_SCHEDULER_IDLE:
        case MODE_GIMBAL_SCHEDULER_RELAX:
            break;
        case MODE_GIMBAL_SCHEDULER_ENCODER_LOCK:
            if (!Mode_Gimbal_StabCalib_Init(
                    &context->encoder_lock, &context->config.encoder_lock)) {
                return false;
            }
            break;
        case MODE_GIMBAL_SCHEDULER_GIMBAL_STAB:
            if (!Mode_Gimbal_GimbalStab_Init(
                    &context->gimbal_stab, &context->config.gimbal_stab)) {
                return false;
            }
            break;
        case MODE_GIMBAL_SCHEDULER_VISION_STAB:
            if (!Mode_Gimbal_VisionStab_Init(
                    &context->vision_stab, &context->config.vision_stab)) {
                return false;
            }
            break;
        case MODE_GIMBAL_SCHEDULER_GRAVITY_CALIB:
            if (!Mode_Gimbal_GravityCalib_Init(
                    &context->gravity_calib,
                    &context->config.gravity_calib)) {
                return false;
            }
            break;
        default:
            return false;
    }
    context->active_mode = mode;
    context->active_mode_valid = true;
    return true;
}

bool Mode_Gimbal_Scheduler_Update(
    Mode_Gimbal_SchedulerContext *context,
    const Mode_Gimbal_SchedulerInput *input,
    Mode_Gimbal_SchedulerOutput *output)
{
    Mode_Gimbal_SchedulerContext next;
    bool entered;

    scheduler_stop_output(output);
    if ((context == NULL) || (input == NULL) || (output == NULL) ||
        !context->initialized || !valid_scheduler_mode(input->requested_mode) ||
        !Mode_Gimbal_Internal_ValidTime(&input->time)) {
        return false;
    }

    next = *context;
    entered = !next.active_mode_valid ||
        (next.active_mode != input->requested_mode);
    if (entered && !enter_mode(&next, input->requested_mode)) {
        return false;
    }

    output->active_mode = next.active_mode;
    output->mode_entered = entered;
    switch (next.active_mode) {
        case MODE_GIMBAL_SCHEDULER_IDLE:
        case MODE_GIMBAL_SCHEDULER_RELAX:
            break;

        case MODE_GIMBAL_SCHEDULER_ENCODER_LOCK:
        {
            Mode_Gimbal_StabCalibInput mode_input;
            Mode_Gimbal_StabCalibOutput mode_output;

            mode_input.time = input->time;
            mode_input.encoder_feedback_valid =
                input->feedback.encoder_feedback_valid;
            mode_input.pitch_encoder_deg =
                input->feedback.pitch_encoder_deg;
            mode_input.yaw_encoder_deg = input->feedback.yaw_encoder_deg;
            mode_input.pitch_motor_speed_deg_per_second =
                input->feedback.pitch_motor_speed_deg_per_second;
            mode_input.yaw_motor_speed_deg_per_second =
                input->feedback.yaw_motor_speed_deg_per_second;
            if (!Mode_Gimbal_StabCalib_Update(
                    &next.encoder_lock, &mode_input, &mode_output)) {
                return false;
            }
            output->current = mode_output.current;
            output->encoder_lock_status = mode_output.status;
            break;
        }

        case MODE_GIMBAL_SCHEDULER_GIMBAL_STAB:
        {
            Mode_Gimbal_GimbalStabInput mode_input;
            Mode_Gimbal_GimbalStabOutput mode_output;

            mode_input.time = input->time;
            mode_input.feedback = input->feedback;
            mode_input.target_valid = input->gimbal_target_valid;
            mode_input.target_pitch_deg = input->gimbal_target_pitch_deg;
            mode_input.target_yaw_deg = input->gimbal_target_yaw_deg;
            if (!Mode_Gimbal_GimbalStab_Update(
                    &next.gimbal_stab, &mode_input, &mode_output)) {
                return false;
            }
            output->current = mode_output.current;
            break;
        }

        case MODE_GIMBAL_SCHEDULER_VISION_STAB:
        {
            Mode_Gimbal_VisionStabInput mode_input;
            Mode_Gimbal_VisionStabOutput mode_output;

            mode_input.time = input->time;
            mode_input.feedback = input->feedback;
            mode_input.vision = input->vision;
            if (!Mode_Gimbal_VisionStab_Update(
                    &next.vision_stab, &mode_input, &mode_output)) {
                return false;
            }
            output->current = mode_output.current;
            output->vision_active = mode_output.vision_active;
            output->vision_scanning = mode_output.scanning;
            break;
        }

        case MODE_GIMBAL_SCHEDULER_GRAVITY_CALIB:
        {
            Mode_Gimbal_GravityCalibInput mode_input;
            Mode_Gimbal_GravityCalibOutput mode_output;

            mode_input.time = input->time;
            mode_input.encoder_feedback_valid =
                input->feedback.encoder_feedback_valid;
            mode_input.pitch_encoder_deg =
                input->feedback.pitch_encoder_deg;
            mode_input.yaw_encoder_deg = input->feedback.yaw_encoder_deg;
            mode_input.pitch_motor_speed_deg_per_second =
                input->feedback.pitch_motor_speed_deg_per_second;
            mode_input.yaw_motor_speed_deg_per_second =
                input->feedback.yaw_motor_speed_deg_per_second;
            mode_input.measured_pitch_current =
                input->feedback.measured_pitch_current;
            if (!Mode_Gimbal_GravityCalib_Update(
                    &next.gravity_calib, &mode_input, &mode_output)) {
                return false;
            }
            output->current = mode_output.current;
            output->gravity_status = mode_output.status;
            output->gravity_sample_ready = mode_output.sample_ready;
            output->gravity_sample = mode_output.sample;
            break;
        }

        default:
            return false;
    }

    *context = next;
    return true;
}
