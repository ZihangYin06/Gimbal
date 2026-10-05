// #include "BoardCom.hpp"
// extern "C" void
// StartBoardComTask(void *argument)
// {
//     uint32_t wake = osKernelGetTickCount();

//     for (;;)
//     {
//         Board_Dr16_Data_Sent();
//         Board_Yaw_Data_Sent();
//         osDelayUntil(wake + 2U);
//         wake += 2U;
//     }
// }