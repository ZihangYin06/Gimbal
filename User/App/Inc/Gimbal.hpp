#ifndef _GIMBAL_HPP
#define _GIMBAL_HPP

#include "cmsis_os.h"
#include "GimbalTask.hpp"
#include "bsp_dr16.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void StartGimbalTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
