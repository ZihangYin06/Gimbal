#ifndef _GIMBAL_HPP
#define _GIMBAL_HPP

#include "cmsis_os.h"
#include "GimbalTask.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

    void StartGimbalTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
