#include "device_init.h"
#include "../BSP/bsp_init.h"
#include "../APP/app_config.h"

Device_LED_t dev_led1;
Device_LED_t dev_led2;
Device_Key_t dev_user_key;

MotorEncoder_t enc_left;
MotorEncoder_t enc_right;
Motor_t motor_left;
Motor_t motor_right;
Bluetooth_Instance_t bt_inst;
JY61P_Instance jy61p_inst;

#include "../BSP/bsp_qei.h"
#include "../BSP/bsp_uart.h"
#include "../APP/broker/broker.h"

static int16_t GetLeftDiff(void) { return BSP_QEI_GetDeltaL(); }
static int16_t GetRightDiff(void) { return BSP_QEI_GetDeltaR(); }

static bool UART_DmaSend(const uint8_t *data, uint16_t len) {
    BSP_UART1_SendDMA((uint8_t*)data, len);
    return true;
}

#include "../BSP/bsp_motor.h"
/* Motor 硬件映射回调 */
static int8_t cur_dir_l = MOTOR_DIR_STOP;
static void MotorL_SetDir(int8_t dir) { cur_dir_l = dir; }
static void MotorL_SetPWM(uint32_t duty) {
    Motor_Dir_t bsp_dir = BSP_MOTOR_DIR_STOP;
    if (cur_dir_l == MOTOR_DIR_FORWARD) bsp_dir = BSP_MOTOR_DIR_FORWARD;
    else if (cur_dir_l == MOTOR_DIR_BACKWARD) bsp_dir = BSP_MOTOR_DIR_BACKWARD;
    // TI 定时器 PWM 默认逻辑：占空比 = (Period - CC) / Period
    // 当前 SysConfig 中 PWM_0 周期(Period)为 1000
    // 所以 CC 值 = 1000 - (duty * 1000) / 100
    uint32_t bsp_duty = 1000 - ((duty * 1000) / 100);
    BSP_Motor_SetState(MOTOR_ID_A, bsp_dir, bsp_duty);
}

static int8_t cur_dir_r = MOTOR_DIR_STOP;
static void MotorR_SetDir(int8_t dir) { cur_dir_r = dir; }
static void MotorR_SetPWM(uint32_t duty) {
    Motor_Dir_t bsp_dir = BSP_MOTOR_DIR_STOP;
    if (cur_dir_r == MOTOR_DIR_FORWARD) bsp_dir = BSP_MOTOR_DIR_FORWARD;
    else if (cur_dir_r == MOTOR_DIR_BACKWARD) bsp_dir = BSP_MOTOR_DIR_BACKWARD;
    // TI 定时器 PWM 默认逻辑：占空比 = (Period - CC) / Period
    // 当前 SysConfig 中 PWM_0 周期(Period)为 1000
    // 所以 CC 值 = 1000 - (duty * 1000) / 100
    uint32_t bsp_duty = 1000 - ((duty * 1000) / 100);
    BSP_Motor_SetState(MOTOR_ID_B, bsp_dir, bsp_duty);
}

/* BSP 函数包装层 (提供给底层回调) */
static void LED1_Write(bool state) { BSP_LED_Set(1, state); }
static void LED1_Toggle(void)      { BSP_LED_Toggle(1); }

static void LED2_Write(bool state) { BSP_LED_Set(2, state); }
static void LED2_Toggle(void)      { BSP_LED_Toggle(2); }

static uint8_t Key_Read(void) {
    return BSP_Switch_Read() ? 1 : 0;
}

void Device_Init(void) {
    Device_LED_Init_Config_t led_cfg;
    led_cfg.InitState = false;          // 初始熄灭

    led_cfg.WritePin  = LED1_Write;
    led_cfg.TogglePin = LED1_Toggle;
    Device_LED_Init(&dev_led1, &led_cfg);

    led_cfg.WritePin  = LED2_Write;
    led_cfg.TogglePin = LED2_Toggle;
    Device_LED_Init(&dev_led2, &led_cfg);

    Device_Key_Init_Config_t key_cfg;
    key_cfg.ReadPin     = Key_Read;
    key_cfg.ActiveLevel = 1;            // BSP_Switch_Read 返回 1 = 按下
    Device_Key_Init(&dev_user_key, &key_cfg);

    Device_WS2812_Init();

    /* 初始化电机编码器 */
    MotorEncoder_Init_Config_t enc_cfg;
    enc_cfg.TotalResolution = CAR_ENCODER_RESOLUTION; 
    enc_cfg.WheelCircumference = CAR_WHEEL_CIRCUMFERENCE_MM;

    enc_cfg.GetPulseDiff = GetLeftDiff;
    MotorEncoder_Init(&enc_left, &enc_cfg);

    enc_cfg.GetPulseDiff = GetRightDiff;
    MotorEncoder_Init(&enc_right, &enc_cfg);

    /* 初始化开环电机驱动 */
    MotorConfig_t m_cfg;
    m_cfg.max_duty = CAR_MOTOR_MAX_DUTY; // 应用层采用 0~100 百分比控制
    m_cfg.dead_zone = 0;
    
    // === 右轮电机 ===
    m_cfg.hw_api.SetDir = MotorL_SetDir;
    m_cfg.hw_api.SetPWM = MotorL_SetPWM;
    Motor_Init(&motor_left, &m_cfg);

    // === 左轮电机 ===
    m_cfg.hw_api.SetDir = MotorR_SetDir;
    m_cfg.hw_api.SetPWM = MotorR_SetPWM;
    Motor_Init(&motor_right, &m_cfg);

    /* 初始化蓝牙/串口通信协议 */
    Bluetooth_Init(&bt_inst, UART_DmaSend);

    /* 初始化 IMU 驱动 */
    JY61P_Init(&jy61p_inst);
}

void Device_Update(void) {
    /* 1. 周期性调用硬件解析/处理任务 (非阻塞) */
    JY61P_Parse_Task(&jy61p_inst);

    /* 2. 更新编码器并上报到总线 (APP_TICK_MS 为基准，这里假设 10ms，暂且用简单差分或等 APP 发起，
          但由于我们需要 dt，最佳做法是由外部传入 dt 或是内部自行测时。这里用固定 0.01f 供演示) */
    MotorEncoder_Update(&enc_left, 0.01f);
    MotorEncoder_Update(&enc_right, 0.01f);
    Broker_Publish_Float(TOPIC_SPEED_LEFT, enc_left.WheelSpeed);
    Broker_Publish_Float(TOPIC_SPEED_RIGHT, enc_right.WheelSpeed);
    Broker_Publish_Float(TOPIC_DISTANCE_LEFT, enc_left.TotalDist);
    Broker_Publish_Float(TOPIC_DISTANCE_RIGHT, enc_right.TotalDist);

    /* 3. 从总线读取占空比，控制底层硬件 */
    float pwm_l = Broker_Subscribe_Float(TOPIC_PWM_LEFT);
    float pwm_r = Broker_Subscribe_Float(TOPIC_PWM_RIGHT);
    
    int32_t motor_en = Broker_Subscribe_Int(TOPIC_MOTOR_ENABLE);
    if (!motor_en) {
        BSP_Motor_Sleep(true); // 物理断电
        pwm_l = 0.0f;
        pwm_r = 0.0f;
    } else {
        BSP_Motor_Sleep(false); // 恢复供电
    }

    /* 极性修正：如果出现正反馈疯跑，请在这里对 pwm_l 或 pwm_r 加负号 */
    Motor_SetDrive(&motor_left,  pwm_l);
    Motor_SetDrive(&motor_right, pwm_r);

    /* 4. 更新按键并上报事件到总线 */
    Device_Key_Update(&dev_user_key, 0.01f);
    Device_Key_Event_e event = Device_Key_GetEvent(&dev_user_key);
    if (event != DEVICE_KEY_EVENT_NONE) {
        Broker_Publish_Int(TOPIC_KEY_EVENT, (int32_t)event);
        Device_Key_ClearEvent(&dev_user_key);
    }
}

void Device_JY61P_TareYaw(void) {
    JY61P_TareYaw(&jy61p_inst);
}

