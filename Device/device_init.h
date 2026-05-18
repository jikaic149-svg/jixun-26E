#ifndef DEVICE_INIT_H
#define DEVICE_INIT_H

#include "device_led.h"
#include "device_key.h"
#include "device_ws2812.h"
#include "motor_encoder.h"
#include "motor.h"
#include "bluetooth.h"
#include "jy61p.h"
/*
 * Device 层 (硬件抽象驱动)
 * 当前最小可工作集：两颗板载 LED + 一个用户按键
 * （WS2812 RGB 已下线，等 DMA+PWM 通道稳定后再接回）
 */

extern Device_LED_t dev_led1;
extern Device_LED_t dev_led2;
extern Device_Key_t dev_user_key;

extern MotorEncoder_t enc_left;
extern MotorEncoder_t enc_right;
extern Motor_t motor_left;
extern Motor_t motor_right;
extern Bluetooth_Instance_t bt_inst;
extern JY61P_Instance jy61p_inst;

void Device_Init(void);
void Device_Update(void);
void Device_JY61P_TareYaw(void);

#endif // DEVICE_INIT_H
