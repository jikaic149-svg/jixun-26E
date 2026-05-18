#ifndef APP_MAIN_H
#define APP_MAIN_H

/* 
 * APP 层 (业务逻辑) 
 * 纯软逻辑，负责订阅数据、算法解算、发布指令。严禁出现具体 MCU 头文件
 */

void App_Init(void);
void App_Task(void);

#endif // APP_MAIN_H
