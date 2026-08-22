#include "INS.hpp"

extern "C" void StartINSTask(void *argument)
{
    (void)argument;

    uint32_t wake = osKernelGetTickCount();

    INS_Init();

    for (;;)
    {
        INS_Task();
        osDelayUntil(wake + 2U);
        wake += 2U;
    }
}