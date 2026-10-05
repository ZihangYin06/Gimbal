#ifndef _BOARD_COM_HPP
#define _BOARD_COM_HPP

#include "cmsis_os.h"
#include "BoardComTask.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

    void StartBoardComTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
