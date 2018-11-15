/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file uart6.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/


#include "uart6.h"
#include "topics.h"
#include "string.h"
#include "scheduler.h"
#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

void uart6_config()
{
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    //PG14------> USART6_TX
    //PG9------> USART6_RX 

    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;

    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    __HAL_RCC_USART6_CLK_ENABLE();

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
    _Error_Handler(__FILE__, __LINE__);
    }
    
    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(&huart6,hdmarx,hdma_usart6_rx);

    /* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(&huart6,hdmatx,hdma_usart6_tx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    /* DMA2_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

//void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
//{

//    if(huart->Instance==USART6) {

//    /* Peripheral clock disable */
//    __HAL_RCC_USART6_CLK_DISABLE();
//  
//    //USART6 GPIO Configuration    
//    //PG14     ------> USART6_TX
//    //PG9     ------> USART6_RX 

//    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

//    /* USART6 interrupt DeInit */
//    HAL_NVIC_DisableIRQ(USART6_IRQn);
//    }

//}

#define LENGTH 100
#define true 1
#define false 0

#define Uart_Debug huart6
extern UART_HandleTypeDef Uart_Debug;
extern void DMA_CLEAR_FLAG_ALL(DMA_HandleTypeDef *dmax);
extern void UART_DMA_RX_INIT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);


//数据拆分宏定义
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

//串口接收缓存
uint8_t uart_rx[LENGTH];

//串口发送缓存
uint8_t uart_tx[LENGTH];

//串口DMA发送标志
int16_t uart_tx_flag = false;

//发送帧帧头
uint8_t txhead = 0;

//发送帧和校验
uint8_t txsum = 0;

//上位机通信校验标志
send_flag_t _send_flag = { false };

//下位机PID缓存
pid_debug_t _pid_debug[18] = { 0 };

//发送数据
send_data_t _send = { 

    .i16_data[0] = 1,
    .i16_data[1] = 2,
    .i16_data[2] = 3,
    .i16_data[3] = 4,

    .f_data[0] = 1.1f,
    .f_data[1] = 2.2f,
    .f_data[2] = 3.3f,
    .f_data[3] = 4.4f,
};

/*********************************串口接收中断*************************************/

void USART6_IRQHandler(void)
{
	if((__HAL_UART_GET_FLAG(&Uart_Debug,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&Uart_Debug);

		__HAL_DMA_DISABLE(Uart_Debug.hdmarx);
		DMA_CLEAR_FLAG_ALL(Uart_Debug.hdmarx);

        //记录接收到的字节数
        uint8_t Num = LENGTH - __HAL_DMA_GET_COUNTER(Uart_Debug.hdmarx);

		do
		{
			if(uart_rx[0] != 0xAA || uart_rx[1] != 0xAF || uart_rx[3] != Num-5)
                break;
			uint8_t num = uart_rx[3] + 5;
			txsum = 0;
			for(uint8_t i = 0;i < Num - 1;i++)
                txsum += uart_rx[i];
			if(txsum != uart_rx[num-1])break;//判断sum
			txhead = uart_rx[2];//帧头
			switch(txhead)  //功能字，判断帧头
			{
				case 0x01U:
//					switch(uart_rx[4])  //校准
//					{
//						case 0x01:gimbal.fric_wheel_run=!gimbal.fric_wheel_run;break;   //ACC校准
//						case 0x02:imu.GyrCal=true;break;    //陀螺仪校准
//						case 0x04:gimbal.trigger_run=!gimbal.trigger_run;break;    //磁力计校准
//						case 0x05:gimbal.shoot_num=1;break;    //气压计校准
//						default:  imu.AccCalFlag=uart_rx[4]-0x20;imu.AccCal=true;break;  //六面校准
//					}
					break;
				case 0x02U:
					switch(uart_rx[4])  //读取
					{
//						case 0x01U://读取pid
//							f.send_pid1 = true;
//							f.send_pid2 = true;
//							f.send_pid3 = true;
//							f.send_pid4 = true;
//							f.send_pid5 = true;
//							f.send_pid6 = true;
//							break;
						default: break;
					}
					break;
				case 0x03U://读取遥控数据
//					f.send_rcdata = true;
					break;
				case 0x10U://写入第1组pid
					Write_PID(&_pid_debug[0],&_pid_debug[1],&_pid_debug[2]);
					break;
				case 0x11U://写入第2组pid
					Write_PID(&_pid_debug[3],&_pid_debug[4],&_pid_debug[5]);
					break;
				case 0x12U://写入第3组pid
					Write_PID(&_pid_debug[6],&_pid_debug[7],&_pid_debug[8]);
					break;
				case 0x13U://写入第4组pid
					Write_PID(&_pid_debug[9],&_pid_debug[10],&_pid_debug[11]);
					break;
				case 0x14U://写入第5组pid
                    Write_PID(&_pid_debug[12],&_pid_debug[13],&_pid_debug[14]);
					break;
				case 0x15U://写入第6组pid
					Write_PID(&_pid_debug[15],&_pid_debug[16],&_pid_debug[17]);
					break;
				default:break;
			}
		}while(0);

		__HAL_DMA_SET_COUNTER(Uart_Debug.hdmarx,LENGTH);

		__HAL_DMA_ENABLE(Uart_Debug.hdmarx);
	}
}

/*****************************串口DMA发送函数*********************************/

void DMA2_Stream6_IRQHandler(void)//uart6 tx
{
	DMA_CLEAR_FLAG_ALL(Uart_Debug.hdmatx);
	__HAL_DMA_DISABLE(Uart_Debug.hdmatx);
    uart_tx_flag = false;
}

void UART_DMA_TX_INIT(UART_HandleTypeDef* huart, uint32_t Size)
{
    __HAL_DMA_DISABLE(huart->hdmatx);
	huart->hdmatx->Instance->PAR = (uint32_t)&huart->Instance->DR;
	huart->hdmatx->Instance->NDTR = Size;
	huart->hdmatx->Instance->M0AR = NULL;
	DMA_CLEAR_FLAG_ALL(huart->hdmatx);
    //开启DMA发送中断
	__HAL_DMA_ENABLE_IT(huart->hdmatx,DMA_IT_TC);
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);
}

int aaa = 0;

int16_t Uart_Transmit_DMA(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
	if(uart_tx_flag == true)
        return true;
	__HAL_DMA_DISABLE(huart->hdmatx);
	huart->hdmatx->Instance->NDTR = Size;
	huart->hdmatx->Instance->M0AR = (uint32_t)pData;
	DMA_CLEAR_FLAG_ALL(huart->hdmatx);
	__HAL_DMA_ENABLE(huart->hdmatx);
	uart_tx_flag=true;
	return true;
}

/******************************上位机初始化函数*******************************/

void debug_init(void)
{
    //使能串口DMA接收并开启串口空闲中断
    UART_DMA_RX_INIT(&Uart_Debug,uart_rx,LENGTH);

    //使能串口DMA发送并开启DMA发送中断
    UART_DMA_TX_INIT(&Uart_Debug,10);

    //初始化发送帧帧头
    uart_tx[0] = 0xAA;
	uart_tx[1] = 0xAA;

    _send_flag.send_check = false;
    _send_flag.send_motopwm = false;
}

/******************************上位机校验函数**********************************/

int16_t Send_Check(void)
{
    if(uart_tx_flag == true)
        return false;
    uart_tx[2] = 0xEF;
	uart_tx[3] = 2;
	uart_tx[4] = txhead;
	uart_tx[5] = txsum;

    uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
        sum += uart_tx[i];
	uart_tx[6]=sum;

	return Uart_Transmit_DMA(&Uart_Debug,(uint8_t *)uart_tx, 7);
}

/*****************************数据发送函数**************************************/

int16_t Send_int16_data(void)
{
    if(uart_tx_flag == true)
        return false;
	uart_tx[2]=0xF1;
	uart_tx[3]=8;

	int16_t temp = _send.i16_data[0];
	uart_tx[4] =BYTE1(temp);
	uart_tx[5] =BYTE0(temp);

	temp = _send.i16_data[1];
	uart_tx[6] =BYTE1(temp);
	uart_tx[7] =BYTE0(temp);

	temp = _send.i16_data[2];
	uart_tx[8] =BYTE1(temp);
	uart_tx[9] =BYTE0(temp);

	temp = _send.i16_data[3];
	uart_tx[10]=BYTE1(temp);
	uart_tx[11]=BYTE0(temp);

	uint8_t sum = 0;
	for(uint8_t i=0;i<12;i++)sum += uart_tx[i];
	uart_tx[12]=sum;

    return Uart_Transmit_DMA(&huart6,(uint8_t *)uart_tx, 13);
}

int16_t Send_float_data(void)
{
    if(uart_tx_flag == true)
        return false;
	uart_tx[2]=0xF1;
	uart_tx[3]=16;

	float temp = _send.f_data[0];
	uart_tx[4] =BYTE3(temp);
	uart_tx[5] =BYTE2(temp);
    uart_tx[6] =BYTE1(temp);
	uart_tx[7] =BYTE0(temp);

	temp = _send.f_data[1];
	uart_tx[8] =BYTE3(temp);
	uart_tx[9] =BYTE2(temp);
    uart_tx[10] =BYTE1(temp);
	uart_tx[11] =BYTE0(temp);

	temp = _send.f_data[2];
	uart_tx[12] =BYTE3(temp);
	uart_tx[13] =BYTE2(temp);
    uart_tx[14] =BYTE1(temp);
	uart_tx[15] =BYTE0(temp);

	temp = _send.f_data[3];
	uart_tx[16] =BYTE3(temp);
	uart_tx[17] =BYTE2(temp);
    uart_tx[18] =BYTE1(temp);
	uart_tx[19] =BYTE0(temp);

	uint8_t sum = 0;
	for(uint8_t i=0;i<20;i++)sum += uart_tx[i];
	uart_tx[20]=sum;

    return Uart_Transmit_DMA(&huart6,(uint8_t *)uart_tx, 21);
}

/******************************写入PID函数*************************************/

void Write_PID(pid_debug_t *pid1,pid_debug_t *pid2,pid_debug_t *pid3)
{
	pid1->p = 0.01f*( (int16_t)(*(uart_rx+4 )<<8)|*(uart_rx+5 ) );
	pid1->i = 0.01f*( (int16_t)(*(uart_rx+6 )<<8)|*(uart_rx+7 ) );
	pid1->d = 0.01f*( (int16_t)(*(uart_rx+8 )<<8)|*(uart_rx+9 ) );
	pid2->p = 0.01f*( (int16_t)(*(uart_rx+10)<<8)|*(uart_rx+11) );
	pid2->i = 0.01f*( (int16_t)(*(uart_rx+12)<<8)|*(uart_rx+13) );
	pid2->d = 0.01f*( (int16_t)(*(uart_rx+14)<<8)|*(uart_rx+15) );
	pid3->p = 0.01f*( (int16_t)(*(uart_rx+16)<<8)|*(uart_rx+17) );
	pid3->i = 0.01f*( (int16_t)(*(uart_rx+18)<<8)|*(uart_rx+19) );
	pid3->d = 0.01f*( (int16_t)(*(uart_rx+20)<<8)|*(uart_rx+21) );

	_send_flag.send_check = !Send_Check();
}

void debug_task(void)
{
    static uint32_t cnt = 0;

    cnt++;

    if((cnt % 5) == 0)  //电机
        _send_flag.send_motopwm = true;

    if(_send_flag.send_check) {
        _send_flag.send_check = !Send_Check();
    }

    else if(_send_flag.send_motopwm) {
//        _send_flag.send_motopwm = !Send_int16_data();
        _send_flag.send_motopwm = !Send_float_data();
    }
	
			//接收第一组pid为pitch内环
        _params.rate_p[1] = _pid_debug[0].p;
        _params.rate_i[1] = _pid_debug[0].i;
        _params.rate_d[1] = _pid_debug[0].d;

		//接收第二组pid为pitch外环
        _params.att_p[1] = _pid_debug[1].p;
//        _params.att_i[1] = _pid_debug[1].i;
//        _params.att_d[1] = _pid_debug[1].d;

		//接收第三组pid为yaw内环
        _params.rate_p[2] = _pid_debug[2].p;
        _params.rate_i[2] = _pid_debug[2].i;
        _params.rate_d[2] = _pid_debug[2].d;

		//接收第四组pid为yaw外环
        _params.att_p[2] = _pid_debug[3].p;
//        _params.att_i[2] = _pid_debug[3].i;
//        _params.att_d[2] = _pid_debug[3].d;

		//接收第五组pid为CM1速度环
        _params.velocity_p[0] = _pid_debug[4].p;
        _params.velocity_i[0] = _pid_debug[4].i;
        _params.velocity_d[0] = _pid_debug[4].d;

		//接收第六组pid为CM2速度环
        _params.velocity_p[1] = _pid_debug[5].p;
        _params.velocity_i[1] = _pid_debug[5].i;
        _params.velocity_d[1] = _pid_debug[5].d;

		//接收第七组pid为CM3速度环
        _params.velocity_p[2] = _pid_debug[6].p;
        _params.velocity_i[2] = _pid_debug[6].i;
        _params.velocity_d[2] = _pid_debug[6].d;

		//接收第八组pid为CM3速度环
        _params.velocity_p[3] = _pid_debug[7].p;
        _params.velocity_i[3] = _pid_debug[7].i;
        _params.velocity_d[3] = _pid_debug[7].d;

    _send.f_data[0] = _attitude.euler[0];
    _send.f_data[1] = _attitude.euler[1];
    _send.f_data[2] = _attitude.euler[2];
}
