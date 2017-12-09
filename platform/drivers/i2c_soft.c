/***************************************************************************
 *
 *  Copyright (c) 2017 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file i2c_soft.c
 *
 * @author zwh <zwh@raaworks.com>
 */

#include "scheduler.h"
#include "stm32f4xx_hal.h"
#include "i2c_soft.h"

// SCL -> PF1
// SDA -> PF0

#define PORT    GPIOF
#define PIN_SCL GPIO_PIN_1
#define PIN_SDA GPIO_PIN_0

#define SCL_H   PORT->BSRR = PIN_SCL
#define SCL_L   PORT->BSRR = PIN_SCL << 16
#define SDA_H   PORT->BSRR = PIN_SDA
#define SDA_L   PORT->BSRR = PIN_SDA << 16
#define SDA     PORT->IDR & PIN_SDA

static void delay()
{
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
	__nop(); __nop(); __nop();
}

void i2c_soft_config()
{
    // GPIO ports clock enable
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = PIN_SCL | PIN_SDA;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(PORT, &GPIO_InitStruct);
}

static int start()
{
	SDA_H;
    SCL_H; delay();

	if(!SDA) {
        // bus busy
        return -1;
    }

	SDA_L; delay();

	if(SDA) {
        // bus bug
        return -1;
    }

	SDA_L; delay();

	return 0;
}

static void stop()
{
	SCL_L; delay();
	SDA_L; delay();
	SCL_H; delay();
	SDA_H; delay();
}

static void ask()
{
	SCL_L; delay();
	SDA_L; delay();
	SCL_H; delay();
	SCL_L; delay();
}

static void noask()
{
	SCL_L; delay();
	SDA_H; delay();
	SCL_H; delay();
	SCL_L; delay();
}

// @return 0-asked -1-noask
static int wait_ask(void)
{
    uint8_t err;

	SCL_L; delay();
	SDA_H; delay();
	SCL_H; delay();

	while (SDA) {
		if (err++ > 50) {
			stop();
			return -1;
		}
	}

	SCL_L; delay();

	return 0;
}

// send 1byte
static void transmit(uint8_t buf)
{
    uint8_t i = 8;

    while(i--) {
        SCL_L; delay();

        if (buf & 0x80) {
            SDA_H;

        } else {
            SDA_L;
        }

        buf <<= 1;

        delay();

        SCL_H; delay();
    }

    SCL_L;
}

// read 1byte
// @ask 0-noask 1-ask
static uint8_t receive(uint8_t with_ask)
{
    uint8_t i=8;
    uint8_t buf=0;

    SDA_H;

    while (i--) {
        buf <<= 1;

        SCL_L; delay();
        SCL_H; delay();

        if(SDA) {
            buf |= 0x01;
        }
    }

    SCL_L;

	if (with_ask) {
		ask();

    } else {
		noask();
    }

    return buf;
}

// write n byte by i2c
int i2c_soft_write(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len)
{	
	start();

	transmit(address << 1); 

	if (wait_ask()) {
        // device do not ask
		stop();
		return -1;
	}

	transmit(reg); 
	wait_ask();

	while (len--) {
		transmit(*buf++); 
		wait_ask();
	}

	stop();

	return 0;
}

// read n byte by i2c
int i2c_soft_read(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len)
{	
	start();

	transmit(address << 1);

	if (wait_ask()) {
        // device don't ask
		stop();
		return -1;
	}

	transmit(reg); 
	wait_ask();

	start();
	transmit(address << 1 | 0x01); 
	wait_ask();

	while (len) {
		if (len == 1) {
			*buf = receive(0);
		} else {
			*buf = receive(1);
		}

		buf++;
		len--;
	}

	stop();

	return 0;
}
