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
#include "uart6.h"
#include "topics.h"
#include "esc820r.h"
#include "escC620.h"
#include "esc6623.h"

#include "control.h"
#include "icm20600.h"
#include "estimator.h"
#include "commander.h"

void timer_callback()
{
    // get accel and gyro
    icm20600_task();

    // get attitude
    estimator_task();

    // calibrate the icm20600 and some of the command
    commander_task();

//    // initialize sbus
    sbus();

    debug_task();

    // get the velocity of wheels
    esc820r_read();
////      escC620_read();

//    // get the angle of gimbal
    esc6623_read();

//    // the main control of chassis and gimbal
    control_task();

    // send command data to esc6623
    esc6623_write();

//    // send command data to esc820r
    esc820r_write();
      escC620_write();
}

void scheduler()
{
    while (1) {
    }
}
