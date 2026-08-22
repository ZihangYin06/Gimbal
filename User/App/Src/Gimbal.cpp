#include "Gimbal.hpp"

extern "C" void StartGimbalTask(void *argument)
{
    uint32_t wake = osKernelGetTickCount();
    Gimbal_Init();

    for (;;)
    {
        Gimbal_Update();
        osDelayUntil(wake + 2U);
        wake += 2U;
    }
}