/*      厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file uart1.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 */

#include "uart1.h"

#include "string.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define LENGTH  18

static uint8_t buffer[LENGTH];

enum state_e {
    STATE_NULL = 0,
    STATE_UPDATED,
    STATE_BUSY
};

struct {
    uint8_t data[LENGTH];
    enum state_e state;
} _uart1 = {
    .state = STATE_NULL
};

UART_HandleTypeDef huart1;

DMA_HandleTypeDef hdma_usart1_rx;

void DMA_CLEAR_FLAG_ALL(DMA_HandleTypeDef *dmax)
{
    uint32_t ele=(uint32_t)dmax->Instance;

    (ele == (uint32_t)DMA1_Stream0)? (DMA1->LIFCR=0x0000003D) :\
    (ele == (uint32_t)DMA1_Stream1)? (DMA1->LIFCR=0x00000F40) :\
    (ele == (uint32_t)DMA1_Stream2)? (DMA1->LIFCR=0x003D0000) :\
    (ele == (uint32_t)DMA1_Stream3)? (DMA1->LIFCR=0x0F400000) :\
    (ele == (uint32_t)DMA1_Stream4)? (DMA1->HIFCR=0x0000003D) :\
    (ele == (uint32_t)DMA1_Stream5)? (DMA1->HIFCR=0x00000F40) :\
    (ele == (uint32_t)DMA1_Stream6)? (DMA1->HIFCR=0x003D0000) :\
    (ele == (uint32_t)DMA1_Stream7)? (DMA1->HIFCR=0x0F400000) :\
    (ele == (uint32_t)DMA2_Stream0)? (DMA2->LIFCR=0x0000003D) :\
    (ele == (uint32_t)DMA2_Stream1)? (DMA2->LIFCR=0x00000F40) :\
    (ele == (uint32_t)DMA2_Stream2)? (DMA2->LIFCR=0x003D0000) :\
    (ele == (uint32_t)DMA2_Stream3)? (DMA2->LIFCR=0x0F400000) :\
    (ele == (uint32_t)DMA2_Stream4)? (DMA2->HIFCR=0x0000003D) :\
    (ele == (uint32_t)DMA2_Stream5)? (DMA2->HIFCR=0x00000F40) :\
    (ele == (uint32_t)DMA2_Stream6)? (DMA2->HIFCR=0x003D0000) :\
    (DMA2->HIFCR=0x0F400000);
}

void UART_DMA_RX_INIT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
    __HAL_DMA_DISABLE(huart->hdmarx);

    huart->hdmarx->Instance->PAR = (uint32_t)&huart->Instance->DR;
    huart->hdmarx->Instance->NDTR = Size;
    huart->hdmarx->Instance->M0AR = (uint32_t)pData;

    DMA_CLEAR_FLAG_ALL(huart->hdmarx);

    __HAL_DMA_ENABLE(huart->hdmarx);

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
}

// USART1_RX -> PB7
// USART1_TX -> PB6
void uart1_config()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 100000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_EVEN;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /* USART1 DMA Init */
    /* USART1_RX Init */
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    UART_DMA_RX_INIT(&huart1, buffer, LENGTH);
}

void USART1_IRQHandler(void)
{
    if ((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)) {

        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        __HAL_DMA_DISABLE(huart1.hdmarx); 

        DMA_CLEAR_FLAG_ALL(huart1.hdmarx);

        if (_uart1.state != STATE_BUSY) {
            memcpy(_uart1.data, buffer, sizeof(_uart1.data));

            _uart1.state = STATE_UPDATED;
        }

        __HAL_DMA_SET_COUNTER(huart1.hdmarx, LENGTH);
        __HAL_DMA_ENABLE(huart1.hdmarx);
    }
}

int uart1_read(uint8_t *buf, int len)
{
    if (_uart1.state == STATE_UPDATED && len == LENGTH) {
        _uart1.state = STATE_BUSY;

        memcpy(buf, _uart1.data, LENGTH);

        _uart1.state = STATE_NULL;

        return 0;

    } else {
        return -1;
    }
}
