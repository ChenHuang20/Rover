/*      厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file topics.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 */

#include "topics.h"

radio_raw_t _radio_raw = { 0 };
radio_t _radio = { 0 };
wheel_velocity_t _wheel_velocity = { 0 };
wheel_velocity_setpoint_t _wheel_velocity_setpoint = { 0 };
