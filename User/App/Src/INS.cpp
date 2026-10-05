#include "INS.hpp"

/**
 * @brief INS 任务入口(APP 层调度):初始化重试 + 500Hz 固定周期调度 TASK 层逻辑
 */
extern "C" void StartINSTask(void *argument)
{
    // 1. 用任务参数中的配置初始化 INS(IMU + EKF)
    //    失败不退出任务:错误码由 TASK 层记录,500ms 后重试(便于排查接线/热插拔)
    const INS_Task_Config_t *cfg = static_cast<const INS_Task_Config_t *>(argument);
    if (cfg == nullptr)
        osThreadExit();

    for (;;)
    {
        if (INS_Init(cfg->hspi, cfg->calibrate))
            break;
        osDelay(500);
    }

    // 2. 500Hz 固定周期解算
    uint32_t wake = osKernelGetTickCount();
    uint32_t dwt_cnt = 0;
    DWT_GetDeltaT(&dwt_cnt); // 丢弃开机以来的累计值,避免首帧把 ~4s 的巨 dt 灌进 EKF

    for (;;)
    {
        float dt = DWT_GetDeltaT(&dwt_cnt);

        // 3. 调用 TASK 层业务逻辑(解算 + 刷新全局快照)
        INS_Update(dt);

        osDelayUntil(wake + 2U);
        wake += 2U;
    }
}
