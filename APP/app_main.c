#include "app_main.h"
#include "app_config.h"
#include "app_init.h"
#include "app_motor.h"
#include "app_comm.h"
#include "app_ui.h"
#include "../Device/device_init.h"
#include "framework/daemon.h"
#include "../Mode/mode_manager.h"

#include "FreeRTOS.h"
#include "task.h"

static void vAppMainTask(void *pvParameters)
{
    (void)pvParameters;

    /* 集中初始化所有的 APP 逻辑模块 */
    App_Modules_Init();
    Mode_Init();

    while (1) {
        /* 0. 更新设备层驱动状态 (如 IMU 解析) */
        Device_Update();

        /* 1. 更新按键及 RGB/LED UI 交互 */
        App_UI_Update(APP_TICK_S);
        
        /* 1.5 运行 Mode 状态机 */
        Mode_Exec();
        
        /* 2. 更新电机编码器测速与里程解算 */
        App_Motor_Update(APP_TICK_S);

        /* 3. 串口通信协议上报 (比如输出左右轮线速度) */
        App_Comm_Update(APP_TICK_MS);

        /* 4. 守护进程更新 (100Hz 监控) */
        DaemonTask();

        /* 任务挂起，等待下一个周期 */
        vTaskDelay(pdMS_TO_TICKS(APP_TICK_MS));
    }
}

void App_Init(void)
{
    // 创建主任务，栈给 1024 应对大量浮点 sprintf 带来的栈消耗
    xTaskCreate(vAppMainTask, "AppMain", 1024, NULL, 1, NULL); 
    vTaskStartScheduler();      /* 进入 FreeRTOS 调度，不再返回 */
}

void App_Task(void)
{
    /* 已交给 FreeRTOS 调度，此函数在 RTOS 下不再被裸机轮询调用 */
}
