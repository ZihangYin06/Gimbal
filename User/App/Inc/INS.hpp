#ifndef _INS_HPP
#define _INS_HPP

#include "cmsis_os.h"
#include "INS_Task.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void StartINSTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
