#ifndef _BSP_SPI_H_
#define _BSP_SPI_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

uint8_t BSP_SPI_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t tx_data);
void BSP_SPI_ReadMulti(SPI_HandleTypeDef *hspi, uint8_t tx_data, uint8_t *rx_buf, uint16_t len);
void BSP_SPI_WriteMulti(SPI_HandleTypeDef *hspi, const uint8_t *tx_buf, uint16_t len);

#endif
