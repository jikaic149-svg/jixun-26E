#include "app_init.h"
#include "app_motor.h"
#include "app_comm.h"
#include "app_ui.h"
#include "broker/broker.h"

void App_Modules_Init(void) {
    /* 初始化消息总线 */
    Broker_Init();
    
    /* 依次初始化各个解耦的 APP 子模块 */
    App_UI_Init();
    App_Motor_Init();
    App_Comm_Init();
}
