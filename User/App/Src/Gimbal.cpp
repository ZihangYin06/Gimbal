// #include "Gimbal.hpp"

// extern "C" void StartGimbalTask(void *argument)
// {
//     uint32_t wake = osKernelGetTickCount();
//     Gimbal_Init();

//     for (;;)
//     {
//         if (dr16_remote.s1 == 1U)
//         {
//             Gimbal_UpDate();
//         }
//         else if (dr16_remote.s1 == 3U)
//         {
//             Gimbal_UpDate();
//         }
//         else if (dr16_remote.s1 == 2U && dr16_remote.s2 == 3U)
//         {
//             Gimbal_UpDate();
//         }
//         else
//         {
//             Gimbal_Standby();
//         }
//         osDelayUntil(wake + 2U);
//         wake += 2U;
//     }
// }