/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file spi5.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#include "spi5.h"
#include "scheduler.h"
#include "stm32f4xx_hal.h"

SPI_HandleTypeDef hspi5;

typedef enum {
    IDLE = 0,
    ICM20600,
    MS5611,
    HMC5983
} device_e;

static device_e get_device(uint8_t address)
{
    device_e device;

    if (address == 0x68) {
        device = ICM20600;

    } else if (address == 0x1e) {
        device = HMC5983;

    } else {
        device = IDLE;
    }

    return device;
}

static void cs(device_e device, uint8_t state)
{
    GPIO_TypeDef* port = 0;
    uint16_t pin = 0;
    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (device) {
        case IDLE:
            break;

        case ICM20600:
            port = GPIOF;
            pin = GPIO_PIN_6;
            break;

        case HMC5983:
            break;

        default:
            break;
    }

    HAL_GPIO_WritePin(port, pin, pin_state);
}

static uint8_t readwritebyte(uint8_t write)
{
    uint8_t read = 0;

    HAL_SPI_TransmitReceive(&hspi5, &write, &read, 1, 1000);

    return read;
}

int spi5_read(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len)
{
    device_e device;

    device = get_device(address);

    // for most devices except MS5611, bit7 is R/W bit.
    // when 0, the data is written into the device.
    // when 1, the data from the device is read.
    if (device != MS5611) {
        reg |= 1 << 7;
    }

    // for HMC5983, bit6 is M/S bit.
    // when 0, the register address will remain unchanged in multiple read/write commands.
    // when 1, the register address will be auto incremented in multiple read/write commands.
    if (device == HMC5983 && len > 1) {
        reg |= 1 << 6;
    }

    // lower CS
    cs(device, 0);

    readwritebyte(reg);

    while (len--) {
        *buf++ = readwritebyte(0xff);
    }

    // higher CS
    cs(device, 1);

    return 0;
}

int spi5_write(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len)
{
    device_e device;

    device = get_device(address);

    // for HMC5983, bit6 is M/S bit.
    // when 0, the register address will remain unchanged in multiple read/write commands.
    // when 1, the register address will be auto incremented in multiple read/write commands.
    if (device == HMC5983 && len > 1) {
        reg |= 1 << 6;
    }

    // lower CS
    cs(device, 0);

    readwritebyte(reg);

    while (len--) {
        readwritebyte(*buf++);
    }

    // higher CS
    cs(device, 1);

    return 0;
}

// SPI1.SCK     -> PF7
// SPI1.MISO    -> PF8
// SPI1.MOSI    -> PF9

// CS.ICM       -> PF6

void spi5_config(void)
{
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_SPI5_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    // SPI gpio configuration
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;

    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // SPI CS configuration
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOF, GPIO_InitStruct.Pin, GPIO_PIN_SET);

    // SPI configuration
    hspi5.Instance = SPI5;
    hspi5.Init.Mode = SPI_MODE_MASTER;
    hspi5.Init.Direction = SPI_DIRECTION_2LINES;
    hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi5.Init.NSS = SPI_NSS_SOFT;
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi5.Init.CRCPolynomial = 7;

    if (HAL_SPI_Init(&hspi5) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_SPI_ENABLE(&hspi5);
}
