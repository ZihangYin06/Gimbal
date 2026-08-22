#include "Shoot.hpp"

extern "C" void StartShootTask(void *argument)
{
    uint32_t wake = osKernelGetTickCount();
    Shoot_Init();

    for (;;)
    {
        Shoot_Update();
        osDelayUntil(wake + 2U);
        wake += 2U;
    }
}