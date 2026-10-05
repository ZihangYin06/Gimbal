// #include "Shoot.hpp"

// extern "C" void StartShootTask(void *argument)
// {
//     uint32_t wake = osKernelGetTickCount();
//     Shoot_Init();

//     for (;;)
//     {
//         if (dr16_remote.s1 == 1U)
//         {
//             Shoot_Update();
//         }
//         else
//         {
//             Shoot_Standby();
//         }
//         Shoot_Update();
//         osDelayUntil(wake + 2U);
//         wake += 2U;
//     }
// }