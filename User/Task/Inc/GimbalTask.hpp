#ifndef _GIMBALTASK_HPP_
#define _GIMBALTASK_HPP_

#include "INSTask.hpp"
#include "bsp_dr16.h"
#include "PID.hpp"
#include "DJI_Motor.hpp"
#include "define.h"

void Gimbal_Init(void);
void Gimbal_UpDate(void);
void Gimbal_Standby(void);

#endif
