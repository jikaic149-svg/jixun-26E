#ifndef APP_MOTOR_H
#define APP_MOTOR_H

void App_Motor_Init(void);
void App_Motor_Update(float dt_s);
float App_Motor_GetSpeedL(void);
float App_Motor_GetSpeedR(void);
float App_Motor_GetPwmL(void);
float App_Motor_GetPwmR(void);

void App_Motor_SetSpeed(float left_pwm, float right_pwm);
void App_Motor_SetTargetSpeed(float left_mm_s, float right_mm_s);
void App_Motor_Stop(void);
void App_Motor_StartIdentSequence(void);

#endif
