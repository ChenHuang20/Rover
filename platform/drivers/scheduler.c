/*      厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file scheduler.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 */

#include "scheduler.h"
#include "main.h"

#include "sbus.h"
#include "esc820r.h"
#include "topics.h"

volatile uint64_t t;

void TimerCallback()
{
    sbus();

    esc820r_read();

    //
    for (int i = 0; i < 4; i++) {
        _wheel_velocity_setpoint.wheel[i] = 0.0f;
    }

    esc820r_write();
}

void scheduler()
{
    while (1) {
        t = time();
    }
}
