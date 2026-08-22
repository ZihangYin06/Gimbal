#include "BoardComTask.hpp"

void Board_Send(uint8_t *data)
{
    CANSend(COM_CAN, COM_TX_ID, data, 8);
}