/*      厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file tim2.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 */

#include "tim2.h"
#include "main.h"
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim2;

void tim2_config()
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 180 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    TIM_MasterConfigTypeDef sMasterConfig;

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    HAL_TIM_Base_Start_IT(&htim2);
}

__weak void TimerCallback(void)
{
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);

    TimerCallback();
}
