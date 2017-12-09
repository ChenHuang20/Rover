/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file scheduler.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#include "scheduler.h"

#include "sbus.h"
#include "esc820r.h"
#include "esc6623.h"
#include "topics.h"
#include "uart6.h"

#include "icm20600.h"
#include "estimator.h"
#include "commander.h"
#include "control.h"

void timer_callback()
{
    icm20600_task();
    estimator_task();

    sbus();
//    esc820r_read();

    commander_task();

    control_task();

//    esc820r_write();
}

void scheduler()
{
    while (1) {
    }
}
