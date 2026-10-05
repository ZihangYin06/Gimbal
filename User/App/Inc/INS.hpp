#ifndef _INS_HPP
#define _INS_HPP

#include "stm32f4xx_hal.h" // SPI_HandleTypeDef
#include <stdbool.h>       // bool(C/C++ 通用)

/* 以下为 C++ 专用内容:freertos.c(C 编译)也会包含本头文件,
 * 它只需 INS_Task_Config_t 和任务入口声明,不能引入 C++ 头 */
#ifdef __cplusplus
#include "INSTask.hpp" // TASK 层业务逻辑接口(INS_Init / INS_Update)
#include "cmsis_os.h"
#include "bsp_dwt.h"   // 用于获取 dt
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /* INS 任务配置(freertos.c 构造后作为任务参数传入) */
    typedef struct
    {
        SPI_HandleTypeDef *hspi;
        bool calibrate;
    } INS_Task_Config_t;

    /* INS 任务入口(APP 层调度):IMU 初始化重试 + 500Hz 周期调度,
     * 业务逻辑在 TASK 层(INSTask.hpp: INS_Init / INS_Update) */
    void StartINSTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
