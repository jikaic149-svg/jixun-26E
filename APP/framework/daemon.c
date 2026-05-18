/**
 * @file    daemon.c
 * @brief   模块离线检测守护进程实现
 * @origin  云台 modules/daemon/daemon.c
 * @note    100%可移植, 已移除 bsp_dwt.h 和 buzzer.h 依赖
 */
#include "daemon.h"
#include <stdlib.h>

static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx = 0;

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    if (idx >= DAEMON_MX_CNT) return NULL;

    DaemonInstance *instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    memset(instance, 0, sizeof(DaemonInstance));

    instance->owner_id = config->owner_id;
    instance->reload_count = (config->reload_count == 0) ? 100 : config->reload_count;
    instance->callback = config->callback;
    instance->temp_count = (config->init_count == 0) ? 100 : config->init_count;
    instance->temp_count = config->reload_count;

    daemon_instances[idx++] = instance;
    return instance;
}

void DaemonReload(DaemonInstance *instance)
{
    instance->temp_count = instance->reload_count;
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return instance->temp_count > 0;
}

void DaemonTask(void)
{
    DaemonInstance *dins;
    for (uint8_t i = 0; i < idx; ++i)
    {
        dins = daemon_instances[i];
        if (dins->temp_count > 0)
            dins->temp_count--;
        else if (dins->callback)
            dins->callback(dins->owner_id);
    }
}
