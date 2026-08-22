#ifndef _SHOOT_HPP_
#define _SHOOT_HPP_

#include "cmsis_os.h"
#include "ShootTask.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

    void StartShootTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif