/***************************************************************************
 *
 *                           厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file main.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089@qq.com>
 *
 ***************************************************************************/

#include "stm32f4xx_hal.h"
#include "core_cm4.h"

#include "scheduler.h"

#include "clock.h"
#include "tim2.h"
#include "can.h"
#include "uart1.h"
#include "uart6.h"
#include "spi4.h"
#include "spi5.h"
#include "i2c_soft.h"
#include "flash.h"

#include "scheduler.h"

#define TickClock   180     // MHz
#define TickPeriod  50000   // us


volatile uint64_t Tick;

__irq void SysTick_Handler()
{
    Tick++;
    HAL_IncTick();
}

// return system boot time in us
uint64_t time()
{
    uint64_t tick = Tick;
    uint64_t val = SysTick->VAL;

    if (tick != Tick) {
        tick = Tick;
        val = SysTick->VAL;
    }

    return (tick + 1) * TickPeriod - val / TickClock;
}

void usleep(uint32_t us)
{
    uint64_t timestamp = time();

    while (time() < timestamp + us);
}

int main()
{
    HAL_Init();

    clock_config();

    flash_read();

    // can1 -> wheel
    can1_config();

    // uart1 -> radio
    uart1_config();

    // uart6 -> radio
    uart6_config();

    // spi4 -> icm20600
    spi4_config();

    // spi5 -> mpu6500
//    spi5_config();

    // software i2c -> mpu6050
    i2c_soft_config();

    // tim2 -> 1ms timer config
    tim2_config();

    SysTick_Config(TickClock * TickPeriod);
//    SysTick_Config(TickClock * 1000);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    debug_init();

    while (1) {
        scheduler();
    }
}

int read(char *path, uint8_t *buf, int len)
{
    int ret = -1;

    switch (path[0]) {
        case 'u':
            if (path[1] == 1) {
                ret = uart1_read(buf, len);
            }
            if (path[1] == 6) {
            }
            break;

        case 'c':
            if (path[1] == 1) {
                ret = can1_read(buf, len);
            }
            break;

        case 's':
            // spi bus
            ret = spi4_read(path[1], path[2], buf, len);
//            ret = spi5_read(path[1], path[2], buf, len);
            break;

        case 'i':
            // i2c bus
            ret = i2c_soft_read(path[1], path[2], buf, len);

        default:
            break;
    }

    return ret;
}

int write(char *path, uint8_t *buf, int len)
{
    int ret = -1;

    switch (path[0]) {
        case 'u':
            if (path[1] == 1) {
            }
            if (path[1] == 6) {
//                ret = uart6_write(buf, len);
            }
            break;

        case 'c':
            if (path[1] == 1) {
                ret = can1_write(buf, len);
            }
            break;

        case 's':
            ret = spi4_write(path[1], path[2], buf, len);
//            ret = spi5_write(path[1], path[2], buf, len);
            break;

        case 'i':
            ret = i2c_soft_write(path[1], path[2], buf, len);
            break;

        default:
            break;
    }

    return ret;
}

void _Error_Handler(char * file, int line)
{
    while (1) {
    }
}
