#include "bsp_spi.h"

uint8_t BSP_SPI_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t tx_data)
{
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(hspi, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

void BSP_SPI_ReadMulti(SPI_HandleTypeDef *hspi, uint8_t tx_data, uint8_t *rx_buf, uint16_t len)
{
    HAL_SPI_TransmitReceive(hspi, &tx_data, rx_buf, len, HAL_MAX_DELAY);
}

void BSP_SPI_WriteMulti(SPI_HandleTypeDef *hspi, const uint8_t *tx_buf, uint16_t len)
{
    HAL_SPI_Transmit(hspi, (uint8_t *)tx_buf, len, HAL_MAX_DELAY);
}