/**
 * @file    daemon.h
 * @brief   模块离线检测守护进程(看门狗模式)
 * @origin  云台 modules/daemon/daemon.h
 * @note    100%可移植, 纯C实现
 *
 * 使用方法:
 *   1. DaemonRegister() 注册需要监控的模块
 *   2. 模块正常工作时调用 DaemonReload() "喂狗"
 *   3. 在定时任务中调用 DaemonTask() (推荐100Hz)
 *   4. 超时未喂狗则自动调用 callback 告警
 */
#ifndef DAEMON_H
#define DAEMON_H

#include <stdint.h>
#include <string.h>

#define DAEMON_MX_CNT 64    // 最大守护实例数量

/* 模块离线处理回调 */
typedef void (*offline_callback)(void *);

/* daemon实例 */
typedef struct
{
    uint16_t reload_count;      // 重载值(喂狗周期)
    offline_callback callback;  // 离线回调函数
    uint16_t temp_count;        // 当前计数, 减为零则离线
    void *owner_id;             // 拥有者标识(通常为模块实例地址)
} DaemonInstance;

/* 初始化配置 */
typedef struct
{
    uint16_t reload_count;      // 喂狗重载值
    uint16_t init_count;        // 初始等待计数(有些模块启动慢)
    offline_callback callback;  // 离线回调
    void *owner_id;             // 拥有者ID
} Daemon_Init_Config_s;

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);
void DaemonReload(DaemonInstance *instance);
uint8_t DaemonIsOnline(DaemonInstance *instance);
void DaemonTask(void);

#endif /* DAEMON_H */
